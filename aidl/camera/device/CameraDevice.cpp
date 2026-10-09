/*
 * Copyright (C) 2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "CamDev-impl"

#include "CameraDevice.h"
#include "convert.h"

#include <cutils/trace.h>

#include <system/camera_metadata.h>

#include <vector>

namespace android {
namespace hardware {
namespace camera {
namespace device {
namespace implementation {

std::string CameraDevice::kDeviceVersion = "1.1";

namespace {

// find_camera_metadata_ro_entry() binary-searches and silently fails on the
// unsorted blobs this HAL emits, so look important keys up by scanning.
bool findEntryByScan(const camera_metadata_t* md, uint32_t tag,
                     camera_metadata_ro_entry_t* entry) {
    const size_t count = get_camera_metadata_entry_count(md);
    for (size_t i = 0; i < count; i++) {
        if (get_camera_metadata_ro_entry(md, i, entry) == 0 && entry->tag == tag) {
            return true;
        }
    }
    return false;
}

// Mutable counterpart: also returns the entry index for update_camera_metadata_entry().
bool findEntryIndexByScan(camera_metadata_t* md, uint32_t tag, size_t* index) {
    const size_t count = get_camera_metadata_entry_count(md);
    camera_metadata_entry_t entry;
    for (size_t i = 0; i < count; i++) {
        if (get_camera_metadata_entry(md, i, &entry) == 0 && entry.tag == tag) {
            *index = i;
            return true;
        }
    }
    return false;
}

// Some Samsung devices assembled by this provider run camera libraries built
// for a different board's camera topology, and for the cameras that do not
// line up the static metadata comes out incomplete or mislabelled.  Two such
// defects are repaired here, both recognised by their pathology rather than
// by camera id so unaffected devices keep their metadata untouched:
//
//  * ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS missing entirely while
//    ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS is present.  The frame
//    duration table lists the (format, width, height) combinations the HAL
//    itself supports, so it is mirrored into a stream configuration table,
//    adding the BLOB sizes from the stall duration table.  Without the table
//    camera clients see "no available output size" and refuse to open the
//    camera (on y2q that is the 64MP telephoto, surfaced as a zoom button
//    that crashes the camera app when tapped).
//
//  * A focal length that is impossible for the lens it belongs to.  The
//    telephoto reports 3.26mm - shorter than the 5.4mm main camera - because
//    the value came from the other board's camera tables; 16.2mm (3.0x the
//    main camera, the 76mm-equivalent module) is reported instead.  The
//    ultra-wide's true 2.2mm focal length would label it "0.4x" relative to
//    the main camera; Samsung calls this lens 0.6x everywhere else, so 0.6 x
//    5.4mm is reported to keep the zoom button honest.  Only labels change;
//    capture metadata and optics are unaffected.
camera_metadata_t* repairSamsungStaticMetadata(camera_metadata_t* src) {
    bool needsStreamConfigs = false;
    camera_metadata_ro_entry_t streamConfigs;
    if (!findEntryByScan(src, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                         &streamConfigs) ||
        streamConfigs.count == 0) {
        needsStreamConfigs = true;
    } else {
        // The table is also unusable when it is present but has no YUV_420_888
        // output entry, which is the format every camera client previews
        // through.  Camera 4 here - the 64MP telephoto - carries exactly four
        // entries, all of them 1280x3840, and no YUV at all, so opening it
        // fails with
        //   java.lang.IllegalArgumentException: No available output size is
        //   found for androidx.camera.core.impl.PreviewConfig
        // which is the zoom button that takes the camera app down with it.
        bool hasYuvOutput = false;
        for (size_t i = 0; i + 3 < streamConfigs.count; i += 4) {
            if (streamConfigs.data.i32[i] == ANDROID_SCALER_AVAILABLE_FORMATS_YCbCr_420_888 &&
                streamConfigs.data.i32[i + 3] ==
                        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT) {
                hasYuvOutput = true;
                break;
            }
        }
        if (!hasYuvOutput) {
            needsStreamConfigs = true;
        }
    }

    // The zoom range is published as 1.0x-8.0x, but the board this HAL
    // metadata was assembled for (and y2q alike) is a 30x-class device: the
    // 64MP telephoto covers 3x optically and Super Resolution zoom takes the
    // crop to 30x.  Extend the range so clients can actually request it.
    //
    // This HAL emits no ANDROID_CONTROL_ZOOM_RATIO_RANGE at all - the framework
    // derives that from ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, which is what
    // the HAL does publish, so rewriting the derived tag alone changes nothing:
    // it is re-derived from the scaler tag on every query.  Both are moved, and
    // the trigger has to accept either one being the 8.0 this metadata carries.
    bool needsZoomRange = false;
    camera_metadata_ro_entry_t zoomRange;
    if (findEntryByScan(src, ANDROID_CONTROL_ZOOM_RATIO_RANGE, &zoomRange) &&
        zoomRange.count == 2 && zoomRange.data.f[1] > 7.9f && zoomRange.data.f[1] < 8.1f) {
        needsZoomRange = true;
    }
    if (!needsZoomRange) {
        camera_metadata_ro_entry_t maxDigital;
        if (findEntryByScan(src, ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &maxDigital) &&
            maxDigital.count == 1 && maxDigital.data.f[0] > 7.9f && maxDigital.data.f[0] < 8.1f) {
            needsZoomRange = true;
        }
    }

    float fixedFocal = 0.0f;
    camera_metadata_ro_entry_t focal;
    if (findEntryByScan(src, ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &focal) &&
        focal.count == 1) {
        if (focal.data.f[0] > 3.25f && focal.data.f[0] < 3.27f) {
            fixedFocal = 16.2f;
        } else if (focal.data.f[0] > 2.19f && focal.data.f[0] < 2.21f) {
            fixedFocal = 3.24f;
        }
    }

    if (!needsStreamConfigs && fixedFocal == 0.0f && !needsZoomRange) {
        return nullptr;
    }

    // Clone into a buffer with room for the additions; the HAL's own blob may
    // not have spare capacity and must not be grown or freed here.
    camera_metadata_t* dst =
            allocate_camera_metadata(get_camera_metadata_entry_count(src) + 8,
                                     get_camera_metadata_data_count(src) + 4096);
    if (dst == nullptr) {
        ALOGE("%s: could not allocate repaired metadata", __FUNCTION__);
        return nullptr;
    }
    if (append_camera_metadata(dst, src) != OK) {
        ALOGE("%s: could not clone static metadata", __FUNCTION__);
        free_camera_metadata(dst);
        return nullptr;
    }

    if (needsStreamConfigs) {
        // availableMinFrameDurations and availableStallDurations are int64[]
        // (nanoseconds), four elements per (format, width, height, duration).
        // The same assembly failure can leave THOSE tables type-mangled, so
        // every mirrored tuple is validated; when too few survive, a standard
        // Samsung size ladder for a 16MP 4:3 binned sensor is declared
        // instead (the 64MP telephoto's binned output is 4000x3000).
        std::vector<int32_t> configs;
        auto pushSize = [&configs](int32_t format, int32_t width, int32_t height) {
            configs.push_back(format);
            configs.push_back(width);
            configs.push_back(height);
            configs.push_back(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT);
        };
        size_t valid = 0;
        // Sizes must fit the sensor's own active array, landscape-oriented
        // the way Samsung publishes them; the mangled duration tables decode
        // to values like 1280x3840 that pass a format check alone.
        camera_metadata_ro_entry_t activeArray;
        int32_t activeW = 0, activeH = 0;
        if (findEntryByScan(src, ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, &activeArray) &&
            activeArray.count == 4) {
            activeW = activeArray.data.i32[2] - activeArray.data.i32[0] + 1;
            activeH = activeArray.data.i32[3] - activeArray.data.i32[1] + 1;
        }
        auto sizePlausible = [activeW, activeH](int32_t w, int32_t h) {
            if (w < 640 || h < 480 || w > 9500 || h > 9500) return false;
            // Samsung publishes these tables landscape; the mangled duration
            // tables decode to portrait pairs like 1280x3840, which pass every
            // other test here - both sides fit the active array - and four of
            // them were enough to satisfy the "valid < 4" check below and keep
            // the size ladder from ever running.
            if (h > w) return false;
            if (activeW > 0 && activeH > 0 && (w > activeW || h > activeH)) return false;
            return true;
        };
        camera_metadata_ro_entry_t durations;
        if (find_camera_metadata_ro_entry(src, ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
                                          &durations) == OK) {
            for (size_t i = 0; i + 3 < durations.count; i += 4) {
                int32_t format = static_cast<int32_t>(durations.data.i64[i]);
                int32_t width = static_cast<int32_t>(durations.data.i64[i + 1]);
                int32_t height = static_cast<int32_t>(durations.data.i64[i + 2]);
                if ((format < 32 || format > 35) || !sizePlausible(width, height)) {
                    continue;
                }
                pushSize(format, width, height);
                valid++;
            }
        }
        camera_metadata_ro_entry_t stalls;
        if (find_camera_metadata_ro_entry(src, ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
                                          &stalls) == OK) {
            constexpr int64_t kBlobFormat = 0x21;
            for (size_t i = 0; i + 3 < stalls.count; i += 4) {
                int32_t format = static_cast<int32_t>(stalls.data.i64[i]);
                int32_t width = static_cast<int32_t>(stalls.data.i64[i + 1]);
                int32_t height = static_cast<int32_t>(stalls.data.i64[i + 2]);
                if (format != kBlobFormat || !sizePlausible(width, height)) {
                    continue;
                }
                pushSize(format, width, height);
                valid++;
            }
        }
        if (valid < 4) {
            configs.clear();
            constexpr int32_t kBlob = 0x21;    // JPEG
            constexpr int32_t kImplDef = 0x22; // preview
            constexpr int32_t kYuv = 0x23;     // analysis
            const int32_t kLadder[][2] = {
                    {4000, 3000}, {4000, 2256}, {3840, 2160}, {2992, 2992},
                    {2400, 1080}, {1920, 1440}, {1920, 1080}, {1440, 1080},
                    {1600, 1200}, {1280, 960},  {1088, 1088},
            };
            for (const auto& size : kLadder) {
                pushSize(kImplDef, size[0], size[1]);
                pushSize(kYuv, size[0], size[1]);
                pushSize(kBlob, size[0], size[1]);
            }
            ALOGI("%s: duration tables unusable (%zu valid tuples), "
                  "declaring standard size ladder",
                  __FUNCTION__, valid);
        }
        if (!configs.empty() &&
            add_camera_metadata_entry(dst, ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                                      configs.data(), configs.size()) == OK) {
            ALOGI("%s: rebuilt stream configuration table with %zu output sizes",
                  __FUNCTION__, configs.size() / 4);
        } else {
            ALOGE("%s: failed to rebuild stream configuration table", __FUNCTION__);
        }
    }

    if (needsZoomRange) {
        float zoomRange[2] = {1.0f, 30.0f};
        size_t zoomIndex;
        if (findEntryIndexByScan(dst, ANDROID_CONTROL_ZOOM_RATIO_RANGE, &zoomIndex) &&
            update_camera_metadata_entry(dst, zoomIndex, zoomRange, 2, nullptr) == OK) {
            ALOGI("%s: extended zoom ratio range to 1.0-30.0", __FUNCTION__);
        }
        float maxDigitalZoom = 30.0f;
        size_t digitalIndex;
        if (findEntryIndexByScan(dst, ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
                                 &digitalIndex)) {
            update_camera_metadata_entry(dst, digitalIndex, &maxDigitalZoom, 1, nullptr);
        } else {
            add_camera_metadata_entry(dst, ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
                                      &maxDigitalZoom, 1);
        }
    }

    if (fixedFocal != 0.0f) {
        size_t focalIndex;
        if (findEntryIndexByScan(dst, ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
                                 &focalIndex) &&
            update_camera_metadata_entry(dst, focalIndex, &fixedFocal, 1, nullptr) == OK) {
            ALOGI("%s: corrected focal length to %f mm", __FUNCTION__, fixedFocal);
        }
    }

    return dst;
}

}  // anonymous namespace

CameraDevice::CameraDevice(
        sp<SamsungCameraModule> module, const std::string& cameraId,
        const SortedVector<std::pair<std::string, std::string>>& cameraDeviceNames)
    : mModule(module),
      mCameraId(cameraId),
      mDisconnected(false),
      mCameraDeviceNames(cameraDeviceNames) {
    mCameraIdInt = atoi(mCameraId.c_str());
    // Should not reach here as provider also validate ID
    if (mCameraIdInt < 0) {
        ALOGE("%s: Invalid camera id: %s", __FUNCTION__, mCameraId.c_str());
        mInitFail = true;
    } else if (mCameraIdInt >= mModule->getNumberOfCameras()) {
        ALOGI("%s: Adding a new camera id: %s", __FUNCTION__, mCameraId.c_str());
    }

    mDeviceVersion = mModule->getDeviceVersion(mCameraIdInt);
    if (mDeviceVersion < CAMERA_DEVICE_API_VERSION_3_2) {
        ALOGE("%s: Camera id %s does not support HAL3.2+", __FUNCTION__, mCameraId.c_str());
        mInitFail = true;
    }
    }

CameraDevice::~CameraDevice() {}

Status CameraDevice::initStatus() const {
    Mutex::Autolock _l(mLock);
    Status status = Status::OK;
    if (mInitFail) {
        status = Status::INTERNAL_ERROR;
    } else if (mDisconnected) {
        status = Status::CAMERA_DISCONNECTED;
    }
    return status;
}

ndk::ScopedAStatus CameraDevice::getCameraCharacteristics(CameraMetadata* _aidl_return) {
    if (_aidl_return == nullptr) {
        return fromStatus(Status::ILLEGAL_ARGUMENT);
    }

    Status status = initStatus();
    if (status == Status::OK) {
        // Module 2.1+ codepath.
        struct camera_info info;
        int ret = mModule->getCameraInfo(mCameraIdInt, &info);
        if (ret == OK && info.static_camera_characteristics != NULL) {
            common::helper::CameraMetadata metadata =
                    (camera_metadata_t*)info.static_camera_characteristics;
            camera_metadata_entry_t entry = metadata.find(ANDROID_FLASH_INFO_AVAILABLE);
            if (entry.count > 0 && *entry.data.u8 != 0 &&
                mModule->isSetTorchModeStrengthSupported()) {
                // Samsung always has 5 supported torch strength levels
                int32_t defaultTorchStrength = 1;
                int32_t torchStrengthLevels = 5;
                metadata.update(ANDROID_FLASH_INFO_STRENGTH_DEFAULT_LEVEL, &defaultTorchStrength,
                                1);
                metadata.update(ANDROID_FLASH_INFO_STRENGTH_MAXIMUM_LEVEL, &torchStrengthLevels, 1);
            }

            // See repairSamsungStaticMetadata(): returns a repaired clone for
            // cameras whose static metadata the HAL mis-assembled, or null when
            // the blob is fine as-is.  The clone is freed after conversion; the
            // HAL's own blob is only ever read here.
            camera_metadata_t* repaired =
                    repairSamsungStaticMetadata((camera_metadata_t*)info.static_camera_characteristics);
                            convertToAidl(repaired != nullptr
                                  ? repaired
                                  : (camera_metadata_t*)info.static_camera_characteristics,
                          _aidl_return);
            if (repaired != nullptr) {
                free_camera_metadata(repaired);
            }
        } else {
            ALOGE("%s: get camera info failed!", __FUNCTION__);
            status = Status::INTERNAL_ERROR;
        }
    }

    return fromStatus(status);
}

ndk::ScopedAStatus CameraDevice::getPhysicalCameraCharacteristics(
        const std::string& in_physicalCameraId, CameraMetadata* _aidl_return) {
    if (_aidl_return == nullptr) {
        return fromStatus(Status::ILLEGAL_ARGUMENT);
    }

    Status status = initStatus();
    if (status == Status::OK) {
        // Require module 2.5+ version.
        if (mModule->getModuleApiVersion() < CAMERA_MODULE_API_VERSION_2_5) {
            ALOGE("%s: get_physical_camera_info must be called on camera module 2.5 or newer",
                  __FUNCTION__);
            status = Status::INTERNAL_ERROR;
        } else {
            char* end;
            errno = 0;
            long id = strtol(in_physicalCameraId.c_str(), &end, 0);
            if (id > INT_MAX || (errno == ERANGE && id == LONG_MAX) || id < INT_MIN ||
                (errno == ERANGE && id == LONG_MIN) || *end != '\0') {
                ALOGE("%s: Invalid physicalCameraId %s", __FUNCTION__, in_physicalCameraId.c_str());
                status = Status::ILLEGAL_ARGUMENT;
            } else {
                camera_metadata_t* physicalInfo = nullptr;
                int ret = mModule->getPhysicalCameraInfo((int)id, &physicalInfo);
                if (ret == OK) {
                    convertToAidl(physicalInfo, _aidl_return);
                } else if (ret == -EINVAL) {
                    ALOGE("%s: %s is not a valid physical camera Id outside of getCameraIdList()",
                          __FUNCTION__, in_physicalCameraId.c_str());
                    status = Status::ILLEGAL_ARGUMENT;
                } else {
                    ALOGE("%s: Failed to get physical camera %s info: %s (%d)!", __FUNCTION__,
                          in_physicalCameraId.c_str(), strerror(-ret), ret);
                    status = Status::INTERNAL_ERROR;
                }
            }
        }
    }

    return fromStatus(status);
}

void CameraDevice::setConnectionStatus(bool connected) {
    Mutex::Autolock _l(mLock);
    mDisconnected = !connected;
    std::shared_ptr<CameraDeviceSession> session = mSession.lock();
    if (session == nullptr) {
        return;
    }
    // Only notify active session disconnect events.
    // Users will need to re-open camera after disconnect event
    if (!connected) {
        session->disconnect();
    }
    return;
}

Status CameraDevice::getAidlStatus(int status) {
    switch (status) {
        case 0:
            return Status::OK;
        case -ENOSYS:
            return Status::OPERATION_NOT_SUPPORTED;
        case -EBUSY:
            return Status::CAMERA_IN_USE;
        case -EUSERS:
            return Status::MAX_CAMERAS_IN_USE;
        case -ENODEV:
            return Status::INTERNAL_ERROR;
        case -EINVAL:
            return Status::ILLEGAL_ARGUMENT;
        default:
            ALOGE("%s: unknown HAL status code %d", __FUNCTION__, status);
            return Status::INTERNAL_ERROR;
    }
}

ndk::ScopedAStatus CameraDevice::getResourceCost(CameraResourceCost* _aidl_return) {
    if (_aidl_return == nullptr) {
        return fromStatus(Status::ILLEGAL_ARGUMENT);
    }

    Status status = initStatus();
    if (status == Status::OK) {
        int cost = 100;
        std::vector<std::string> conflicting_devices;
        struct camera_info info;

        // If using post-2.4 module version, query the cost + conflicting devices from the HAL
        if (mModule->getModuleApiVersion() >= CAMERA_MODULE_API_VERSION_2_4) {
            int ret = mModule->getCameraInfo(mCameraIdInt, &info);
            if (ret == OK) {
                cost = info.resource_cost;
                for (size_t i = 0; i < info.conflicting_devices_length; i++) {
                    std::string cameraId(info.conflicting_devices[i]);
                    for (const auto& pair : mCameraDeviceNames) {
                        if (cameraId == pair.first) {
                            conflicting_devices.push_back(pair.second);
                        }
                    }
                }
            } else {
                status = Status::INTERNAL_ERROR;
            }
        }

        if (status == Status::OK) {
            _aidl_return->resourceCost = cost;
            _aidl_return->conflictingDevices.resize(conflicting_devices.size());
            for (size_t i = 0; i < conflicting_devices.size(); i++) {
                _aidl_return->conflictingDevices[i] = conflicting_devices[i];
                ALOGV("CamDevice %s is conflicting with camDevice %s", mCameraId.c_str(),
                      _aidl_return->conflictingDevices[i].c_str());
            }
        }
    }

    return fromStatus(status);
}

ndk::ScopedAStatus CameraDevice::isStreamCombinationSupported(const StreamConfiguration& in_streams,
                                                              bool* _aidl_return) {
    Status status;
    *_aidl_return = false;

    // Require module 2.5+ version.
    if (mModule->getModuleApiVersion() < CAMERA_MODULE_API_VERSION_2_5) {
        ALOGE("%s: is_stream_combination_supported must be called on camera module 2.5 or "
              "newer",
              __FUNCTION__);
        status = Status::INTERNAL_ERROR;
    } else {
        camera_stream_combination_t streamComb{};
        streamComb.operation_mode = static_cast<uint32_t>(in_streams.operationMode);
        streamComb.num_streams = in_streams.streams.size();
        camera_stream_t* streamBuffer = new camera_stream_t[streamComb.num_streams];

        size_t i = 0;
        for (const auto& it : in_streams.streams) {
            streamBuffer[i].stream_type = static_cast<int>(it.streamType);
            streamBuffer[i].width = it.width;
            streamBuffer[i].height = it.height;
            streamBuffer[i].format = static_cast<int>(it.format);
            streamBuffer[i].data_space = static_cast<android_dataspace_t>(it.dataSpace);
            streamBuffer[i].usage = static_cast<uint32_t>(it.usage);
            streamBuffer[i].physical_camera_id = it.physicalCameraId.c_str();
            streamBuffer[i++].rotation = static_cast<int>(it.rotation);
        }
        streamComb.streams = streamBuffer;
        auto res = mModule->isStreamCombinationSupported(mCameraIdInt, &streamComb);
        switch (res) {
            case NO_ERROR:
                *_aidl_return = true;
                status = Status::OK;
                break;
            case BAD_VALUE:
                status = Status::OK;
                break;
            case INVALID_OPERATION:
                status = Status::OPERATION_NOT_SUPPORTED;
                break;
            default:
                ALOGE("%s: Unexpected error: %d", __FUNCTION__, res);
                status = Status::INTERNAL_ERROR;
        };
        delete[] streamBuffer;
    }

    return fromStatus(status);
}

ndk::ScopedAStatus CameraDevice::open(const std::shared_ptr<ICameraDeviceCallback>& in_callback,
                                      std::shared_ptr<ICameraDeviceSession>* _aidl_return) {
    if (_aidl_return == nullptr) {
        ALOGE("%s: cannot open camera %s. return session ptr is null!", __FUNCTION__,
              mCameraId.c_str());
        return fromStatus(Status::ILLEGAL_ARGUMENT);
    }

    Status status = initStatus();
    std::shared_ptr<CameraDeviceSession> session = nullptr;

    if (in_callback == nullptr) {
        ALOGE("%s: cannot open camera %s. callback is null!", __FUNCTION__, mCameraId.c_str());
        return fromStatus(Status::ILLEGAL_ARGUMENT);
    }

    if (status != Status::OK) {
        // Provider will never pass initFailed device to client, so
        // this must be a disconnected camera
        ALOGE("%s: cannot open camera %s. camera is disconnected!", __FUNCTION__,
              mCameraId.c_str());
        return fromStatus(Status::CAMERA_DISCONNECTED);
    } else {
        mLock.lock();

        ALOGV("%s: Initializing device for camera %d", __FUNCTION__, mCameraIdInt);
        session = mSession.lock();
        if (session != nullptr && !session->isClosed()) {
            ALOGE("%s: cannot open an already opened camera!", __FUNCTION__);
            mLock.unlock();
            return fromStatus(Status::CAMERA_IN_USE);
        }

        /** Open HAL device */
        status_t res;
        camera3_device_t* device;

        ATRACE_BEGIN("camera3->open");
        res = mModule->open(mCameraId.c_str(), reinterpret_cast<hw_device_t**>(&device));
        ATRACE_END();

        if (res != OK) {
            ALOGE("%s: cannot open camera %s!", __FUNCTION__, mCameraId.c_str());
            mLock.unlock();
            return fromStatus(getAidlStatus(res));
        }

        /** Cross-check device version */
        if (device->common.version < CAMERA_DEVICE_API_VERSION_3_2) {
            ALOGE("%s: Could not open camera: "
                  "Camera device should be at least %x, reports %x instead",
                  __FUNCTION__, CAMERA_DEVICE_API_VERSION_3_2, device->common.version);
            device->common.close(&device->common);
            mLock.unlock();
            return fromStatus(Status::ILLEGAL_ARGUMENT);
        }

        struct camera_info info;
        res = mModule->getCameraInfo(mCameraIdInt, &info);
        if (res != OK) {
            ALOGE("%s: Could not open camera: getCameraInfo failed", __FUNCTION__);
            device->common.close(&device->common);
            mLock.unlock();
            return fromStatus(Status::ILLEGAL_ARGUMENT);
        }

        session = createSession(device, info.static_camera_characteristics, in_callback);
        if (session == nullptr) {
            ALOGE("%s: camera device session allocation failed", __FUNCTION__);
            mLock.unlock();
            return fromStatus(Status::INTERNAL_ERROR);
        }
        if (session->isInitFailed()) {
            ALOGE("%s: camera device session init failed", __FUNCTION__);
            session = nullptr;
            mLock.unlock();
            return fromStatus(Status::INTERNAL_ERROR);
        }
        mSession = session;

        mLock.unlock();
    }

    *_aidl_return = session;
    return fromStatus(Status::OK);
}

ndk::ScopedAStatus CameraDevice::openInjectionSession(const std::shared_ptr<ICameraDeviceCallback>&,
                                                      std::shared_ptr<ICameraInjectionSession>*) {
    return fromStatus(Status::OPERATION_NOT_SUPPORTED);
}

ndk::ScopedAStatus CameraDevice::setTorchMode(bool in_on) {
    if (!mModule->isSetTorchModeSupported()) {
        return fromStatus(Status::OPERATION_NOT_SUPPORTED);
    }

    Status status = initStatus();
    if (status == Status::OK) {
        if (in_on && mModule->isSetTorchModeStrengthSupported()) {
            return turnOnTorchWithStrengthLevel(mTorchStrengthLevel);
        }

        status = getAidlStatus(mModule->setTorchMode(mCameraId.c_str(), in_on));
        if (!in_on && status == Status::OK) {
            // The API dictates that whenever the torch is turned off,
            // the brightness should reset to the default level
            mTorchStrengthLevel = 1;
        }
    }
    return fromStatus(status);
}

ndk::ScopedAStatus CameraDevice::turnOnTorchWithStrengthLevel(int32_t in_torchStrength) {
    if (!mModule->isSetTorchModeStrengthSupported()) {
        return fromStatus(Status::OPERATION_NOT_SUPPORTED);
    }

    Status status = initStatus();
    if (status == Status::OK) {
        status = getAidlStatus(
                mModule->setTorchModeStrength(mCameraId.c_str(), true, in_torchStrength));
        if (status == Status::OK) {
            mTorchStrengthLevel = in_torchStrength;
        }
    }
    return fromStatus(status);
}

ndk::ScopedAStatus CameraDevice::getTorchStrengthLevel(int32_t* _aidl_return) {
    if (!mModule->isSetTorchModeSupported()) {
        return fromStatus(Status::OPERATION_NOT_SUPPORTED);
    }

    *_aidl_return = mTorchStrengthLevel;
    return fromStatus(Status::OK);
}

std::shared_ptr<CameraDeviceSession> CameraDevice::createSession(
        camera3_device_t* device, const camera_metadata_t* deviceInfo,
        const std::shared_ptr<ICameraDeviceCallback>& callback) {
    return ndk::SharedRefBase::make<CameraDeviceSession>(device, deviceInfo, callback);
}

binder_status_t CameraDevice::dump(int fd, const char** args, uint32_t numArgs) {
    std::shared_ptr<CameraDeviceSession> session = mSession.lock();
    if (session == nullptr) {
        dprintf(fd, "No active camera device session instance\n");
        return STATUS_OK;
    }

    return session->dump(fd, args, numArgs);
}

}  // namespace implementation
}  // namespace device
}  // namespace camera
}  // namespace hardware
}  // namespace android
