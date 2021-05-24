LOCAL_PATH := $(call my-dir)

ifneq ($(TARGET_SURVIVE_MATH_BACKEND),)
    SURVIVE_MATH_BACKEND := $(TARGET_SURVIVE_MATH_BACKEND)
else
    SURVIVE_MATH_BACKEND := eigen
endif

COMMON_CFLAGS := \
    -DBUILD_LH1_SUPPORT \
    -DSURVIVE_LIBUSB_UNVER_DIR \
    -Wno-error=unused-function \
    -Wno-error=unused-parameter \
    -Wno-error=unused-variable \
    -Wno-error=typedef-redefinition \
    -Wno-error=ignored-qualifiers \
    -Wno-error=switch \
    -Wno-error=missing-braces \
    -Wno-error=absolute-value \
    -Wno-error=sign-compare \
    -Wno-error=format \
    -Wno-error=parentheses \
    -Wno-error=unknown-pragmas

COMMON_C_INCLUDES := \
    $(LOCAL_PATH)/include/libsurvive \
    $(LOCAL_PATH)/redist

ifeq ($(SURVIVE_MATH_BACKEND),eigen)
    COMMON_CFLAGS += -DUSE_EIGEN
endif

include $(CLEAR_VARS)
LOCAL_MODULE             := libsurvive
LOCAL_MODULE_CLASS       := SHARED_LIBRARIES
LOCAL_CFLAGS             := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES         := $(COMMON_C_INCLUDES)
LOCAL_PROPRIETARY_MODULE := true

LOCAL_EXPORT_C_INCLUDE_DIRS := \
    $(LOCAL_PATH)/include \
    $(LOCAL_PATH)/redist

LOCAL_SHARED_LIBRARIES := \
    libusb \
    libutils \
    libz

LOCAL_SRC_FILES := \
    redist/mpfit/mpfit.c \
    redist/crc32.c \
    redist/glutil.c \
    redist/jsmn.c \
    redist/json_helpers.c \
    redist/linmath.c \
    redist/puff.c \
    src/barycentric_svd/barycentric_svd.c \
    src/lfsr.c \
    src/lfsr_lh2.c \
    src/ootx_decoder.c \
    src/poser.c \
    src/poser_general_optimizer.c \
    src/survive.c \
    src/survive_api.c \
    src/survive_async_optimizer.c \
    src/survive_config.c \
    src/survive_default_devices.c \
    src/survive_disambiguator.c \
    src/survive_driverman.c \
    src/survive_kalman.c \
    src/survive_kalman_tracker.c \
    src/survive_optimizer.c \
    src/survive_recording.c \
    src/survive_plugins.c \
    src/survive_process.c \
    src/survive_process_gen1.c \
    src/survive_process_gen2.c \
    src/survive_reproject.c \
    src/survive_reproject_gen2.c \
    src/survive_sensor_activations.c \
    src/survive_str.c

ifeq ($(SURVIVE_MATH_BACKEND),eigen)
    LOCAL_HEADER_LIBRARIES := libeigen
    LOCAL_SRC_FILES += \
        redist/sv_matrix.c \
        redist/sv_matrix.eigen.cpp
endif

ifneq ($(TARGET_SURVIVE_CONFIG_PATH),)
    LOCAL_CFLAGS += -DSURVIVE_CONFIG_PATH=\"$(TARGET_SURVIVE_CONFIG_PATH)\"
endif

include $(BUILD_SHARED_LIBRARY)

# driver_vive
include $(CLEAR_VARS)
LOCAL_MODULE               := libsurvive-plugin-driver_vive
LOCAL_MODULE_CLASS         := SHARED_LIBRARIES
LOCAL_CFLAGS               := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES           := $(COMMON_C_INCLUDES)
LOCAL_SHARED_LIBRARIES     := libsurvive libusb
LOCAL_SRC_FILES            := src/driver_vive.c
LOCAL_PROPRIETARY_MODULE   := true
LOCAL_MODULE_RELATIVE_PATH := libsurvive/plugins
include $(BUILD_SHARED_LIBRARY)

# disambiguator_statebased
include $(CLEAR_VARS)
LOCAL_MODULE               := libsurvive-plugin-disambiguator_statebased
LOCAL_MODULE_CLASS         := SHARED_LIBRARIES
LOCAL_CFLAGS               := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES           := $(COMMON_C_INCLUDES)
LOCAL_SHARED_LIBRARIES     := libsurvive
LOCAL_SRC_FILES            := src/disambiguator_statebased.c
LOCAL_PROPRIETARY_MODULE   := true
LOCAL_MODULE_RELATIVE_PATH := libsurvive/plugins
include $(BUILD_SHARED_LIBRARY)

# poser_barycentric_svd
include $(CLEAR_VARS)
LOCAL_MODULE               := libsurvive-plugin-poser_barycentric_svd
LOCAL_MODULE_CLASS         := SHARED_LIBRARIES
LOCAL_CFLAGS               := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES           := $(COMMON_C_INCLUDES)
LOCAL_SHARED_LIBRARIES     := libsurvive
LOCAL_SRC_FILES            := src/poser_barycentric_svd.c
LOCAL_PROPRIETARY_MODULE   := true
LOCAL_MODULE_RELATIVE_PATH := libsurvive/plugins
include $(BUILD_SHARED_LIBRARY)

# poser_mpfit
include $(CLEAR_VARS)
LOCAL_MODULE               := libsurvive-plugin-poser_mpfit
LOCAL_MODULE_CLASS         := SHARED_LIBRARIES
LOCAL_CFLAGS               := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES           := $(COMMON_C_INCLUDES)
LOCAL_SHARED_LIBRARIES     := libsurvive
LOCAL_SRC_FILES            := src/poser_mpfit.c
LOCAL_PROPRIETARY_MODULE   := true
LOCAL_MODULE_RELATIVE_PATH := libsurvive/plugins
include $(BUILD_SHARED_LIBRARY)

# survive-cli
include $(CLEAR_VARS)
LOCAL_MODULE               := survive-cli
LOCAL_MODULE_CLASS         := EXECUTABLES
LOCAL_CFLAGS               := $(COMMON_CFLAGS)
LOCAL_C_INCLUDES           := $(COMMON_C_INCLUDES)
LOCAL_SHARED_LIBRARIES     := libsurvive
LOCAL_SRC_FILES            := survive-cli.c
LOCAL_PROPRIETARY_MODULE   := true
include $(BUILD_EXECUTABLE)
