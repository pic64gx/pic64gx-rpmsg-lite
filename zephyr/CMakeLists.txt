cmake_minimum_required (VERSION 3.5)

if(CONFIG_RPMSGLITE)
  set(ZEPHYR_BUILD 1)
  set(ZEPHYR_BOARD ${CONFIG_BOARD})

  set(RPMSGLITE_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/lib)

  project (rpmsglite C)

  if("${CONFIG_SOC_SERIES}" STREQUAL "polarfire")
    set(CONFIG_SOC polarfire)
  endif()

  if("${CONFIG_SOC_SERIES}" STREQUAL "pic64gx")
    set(CONFIG_SOC pic64gx)
  endif()

  # Get MCUX_DEVICE and MCUX_DEVICE_PATH from CONFIG_SOC
  # For example, if CONFIG_SOC=mimx8ml8_ca53, then MCUX_DEVICE=MIMX8ML8_ca53
  # and MCUX_DEVICE_PATH=MIMX8ML8
  string(FIND ${CONFIG_SOC} "_" pos)
  if(pos GREATER -1)
    string(REPLACE "_" ";" MCUX_DEVICE_TMP ${CONFIG_SOC})
    list(GET MCUX_DEVICE_TMP 0 MCUX_DEVICE_SOC)
    list(GET MCUX_DEVICE_TMP 1 MCUX_DEVICE_CORE)
    string(TOUPPER ${MCUX_DEVICE_SOC} MCUX_DEVICE_PATH)
    set(MCUX_DEVICE ${MCUX_DEVICE_PATH}_${MCUX_DEVICE_CORE})
  else()
    string(TOUPPER ${CONFIG_SOC} MCUX_DEVICE)
    string(TOUPPER ${CONFIG_SOC} MCUX_DEVICE_PATH)
  endif()

  #
  # Select only supported platforms for RPMSG-Lite based on Zephyr selected SOC.
  #
  if(("${MCUX_DEVICE}" STREQUAL "LPC54114") OR ("${MCUX_DEVICE}" STREQUAL "LPC54114_m0"))
    set(RPMSG_LITE_PLATFORM_PATH "lpc5411x")
  elseif("${MCUX_DEVICE}" STREQUAL "LPC55S69_cpu0" OR "${MCUX_DEVICE}" STREQUAL "LPC55S69_cpu1")
    set(RPMSG_LITE_PLATFORM_PATH "lpc55s69")
  elseif("${MCUX_DEVICE}" STREQUAL "MIMXRT1176_cm4" OR "${MCUX_DEVICE}" STREQUAL "MIMXRT1176_cm7")
    set(RPMSG_LITE_PLATFORM_PATH "imxrt1170")
  elseif("${MCUX_DEVICE}" STREQUAL "MIMXRT1166_cm4" OR "${MCUX_DEVICE}" STREQUAL "MIMXRT1166_cm7")
    set(RPMSG_LITE_PLATFORM_PATH "imxrt1160")
  elseif("${MCUX_DEVICE}" STREQUAL "POLARFIRE")
    set(RPMSG_LITE_PLATFORM_PATH "polarfire")
  elseif("${MCUX_DEVICE}" STREQUAL "PIC64GX")
    set(RPMSG_LITE_PLATFORM_PATH "pic64gx")
  else()
    message(FATAL_ERROR "RPMSG-Lite Selected for unsupported platform: ${CONFIG_SOC}")
  endif()


  set (RPMSGLITE_LIB rpmsg_lite)

  if (ZEPHYR_BUILD)
    #
    # Main RPMSG-Lite Sources
    #
    list(APPEND _SOURCES
      ${RPMSGLITE_DIR}/common/llist.c
      ${RPMSGLITE_DIR}/rpmsg_lite/rpmsg_lite.c
      ${RPMSGLITE_DIR}/virtio/virtqueue.c
    )

    #
    # Platform Specific RPMSG-Lite Sources
    #
    list(APPEND _SOURCES
      ${RPMSGLITE_DIR}/rpmsg_lite/porting/platform/${RPMSG_LITE_PLATFORM_PATH}/rpmsg_platform_zephyr_ipm.c
    )

    #
    # Environment Specific RPMSG-Lite Sources
    #
    list(APPEND _SOURCES
      ${RPMSGLITE_DIR}/rpmsg_lite/porting/environment/rpmsg_env_zephyr.c
    )

    zephyr_library_named(${RPMSGLITE_LIB})
    add_dependencies(${ZEPHYR_CURRENT_LIBRARY} ${OFFSETS_H_TARGET})
    zephyr_library_sources(${_SOURCES})
    zephyr_include_directories(
      ${RPMSGLITE_DIR}/include/platform/${RPMSG_LITE_PLATFORM_PATH}
      ${RPMSGLITE_DIR}/include/environment/zephyr
      ${RPMSGLITE_DIR}/include
      )

    zephyr_library_sources_ifdef(CONFIG_RPMSGLITE_QUEUE  ${RPMSGLITE_DIR}/rpmsg_lite/rpmsg_queue.c)
    zephyr_library_sources_ifdef(CONFIG_RPMSGLITE_NS  ${RPMSGLITE_DIR}/rpmsg_lite/rpmsg_ns.c)

  else(ZEPHYR_BUILD)
    # other way from MCUXSDK?
  endif (ZEPHYR_BUILD)
endif(CONFIG_RPMSGLITE)
