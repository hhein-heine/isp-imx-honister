NXP_V5.10_P16:
	bugfix:
		(1)M865SW-767: fix ov2775 set gain lead to bad frame 
		(2)M865SW-770: fix purple edge on left when preview video
		(3)BSLRM865-159: fix bad frame comes up on 4K HDR unit test
		(4)BSLRM865-108\M865SW-828:fix acquiring RAW12 image data on v4l2 interface
			v4l2-ctl -d /dev/video2 --verbose --set-fmt-video=width=1920,height=1080,pixelformat=BA12 --stream-mmap --stream-count=1 --stream-to=1080P.raw
		(5)M865SW-519: fix isp_media_server crash when shift between 2 camera APKs
		(6)BSLRM865-174: fix vvext bls error 
		(7)BSLRM865-118:fixswitching dewarper mirror x/y is not synchronized to frame
		(8)BSLRM865-175: fix Tuning tools : set ae to 0, and the image brightness can not revert until be set to 40
		(9)BSLRM865-179: fix tuning user gamma bug
		(10)M865SW-835: fix Some XML parameter seems to be invalid when use gstreamer command
		(11)BSLRM865-180: vvcam_isp sporadic driver load failure due to failing clk setup

	enhancement:
		(1)M865SW-787: remove directory appshell/native
		(2)M865SW-714: add type 4(wayland) for dual camera dispaly
			./video_test -w 1920 -h 1080 -f NV12 -t 4 -d 2
			./video_test -w 1920 -h 1080 -f NV12 -t 4 -d 2
		(3)M865SW-813: merge patch to enable GKI
		(4)M865SW-619:Refine the log to indicate more valuable informaiton for debug
		(5)M865SW-831:merge Android patches for isp-4.2.2p15
		(6)BSLRM865-176: fix tuing select sensor test pattern
		(7)BSLRM865-158: add dewarp Get and Set api
		(8)M865SW-750\M865SW-823: afps support
		(9)M865SW-788: remove VIV_VIDIOC_QUERY_EXTMEM define in units/fpga/fpga/source/altera_fpga.c
		(10)M865SW-715: video_test  support arbitrary resolution
		(11)M865SW-760: support 'Capture' function in vvext
		(12)M865SW-800: Merge android patches for 4.2.2p13
		(13)BSLRM865-121: cleaning up tuning-client
		(14) android support dual 2775

	known issue:
		(1)BSLRM865-191:basler sensor set fps have yellow mask
		(2)BSLRM865-191:basler sensor set pipeline on/off may stuck
		(3)BSLRM865-188:fps control not so accuracy to pass testEdgeModeControl
		(4)BSLRM865-163:the video jitter on 4k screen
		(5)M865SW-621: on dual ISP camera case, first camera can't work if second camera module not loaded
		(6)M865SW-815: Tuning tools display is not coming out good for non-standard resolution
		(7)gstream loss frame
		(8)M865SW-836:video have yellow mask at beginnings.

NXP_V5.15_P17:
	bugfix:
		(1)M865SW-856:ISP: compiler problem patch integration
		(2)M865SW-836:4k30 unit test capture first frames color is abnormal.
		(3)M865SW-844:Question related to color processing option in MF
		(4)BSLRM865-189:Image quality with HDR mode is worse than Linear mode
		(5)M865SW-815:[ISP/Calibration] Tuning tools display is not coming out good.
		(6)M865SW-841:[imx8mp]resize sensor output image by acquisition window
		(7)M865SW-868:To set wide ClippingPolygon, AWB seems freeze in specific scene.
		(8)BSLRM865-186:tuning tool: Capture image quality is worse than normal V4L2 mode
		(9)BSLRM865-182:Wrong Scale in Exposure Diagramme
		(10)M865SW-812:ISP output size limitation
		(11)M865SW-840:The vvcam-video driver VIDIOC_S_CROP does not to be a standard v4l2 specification
		(12)M865SW-870:alignment issue when bypass dewarp
	enhancement:
		(1)BSLRM865-200:Vvcam_module: sensor_driver: Sensor api changes because of 5.15 linux kernel API change
		(2)M865SW-855:HDR image quality is too dark
		(3)M865SW-854:issue when v4l2-ctl command to get raw10 data
		(4)M865SW-847:negative test: 'vvext -v' causes vvext acts on wrong video device
		(5)BSLRM865-196:use Linux 5.15 dev environment for ISP P17
		(6)M865SW-869:remove basler's driver installation from cmake
		(7)M865SW-630:request to build cpp-netlib to a share lib for tuning-server
	known issue:
		(1)BSLRM865-211:basler sensor set fps have yellow mask
		(2)BSLRM865-210:Basler sensor set pipeline on/off will print error log
		(3)BSLRM865-209:Basler sensor with mode 2/3 have  no effect about BLS when test  tuning tool and vvext feature.
		(4)BSLRM865-212:video test, capture YUV picture, the color is not normal until the 20th frame
		(5)setting hdr  parameters, video seems no effect.
		(6)Board and video may hang when use "ctrl+c" to stop  tuning server after capture picture by tuning client.
		(7)Camera stress test, fail to hear voice
		(8)BSLRM865-163:Basler/ov2775/oa08a20 sensor, the video jitter on 4k screen.
		(9)M865SW-621: on dual ISP camera case, first camera can't work if second camera module not loaded
		(10)Basler_ov5640, basler¡¯s awb Inaccurate colors
		(11)Android_CTS test, dual ov2775 fail 68 cases, dual basler fail 48 cases. please check test report for the details.
		(12)Ov2775  gstreamer test, lost frames.
		(13)BSLRM865-188:fps control not so accuracy to pass test Edge Mode Control
		(14)The objective and subjective quality of the image is not very good, please refer to the test report

NXP_V5.15_P18:
	bugfix:
		(1)M865SW-862: can not close AE completely
		(2)M865SW-892: can not get sensor mode through VIV_VIDIOC_GET_CAPS_SUPPORTS
		(3)M865SW-833: ISP AEC shock: The brightness shock in some scene
		(4)M865SW-826: ISP 3A: AEC/GC effective mode is missed
		(5)M865SW-834: testPreviewPictureSizesCombination cause block in deque buffer on 720p
		(6)M865SW-872: tuning tools "change driver" several times, met the mistake "wrong state"
		(7)M865SW-818: implement/verify AF function with AR1335
		(8)M865SW-751: Provide awb/aec lock and get status function
		(9)M865SW-629: AF function worked basically in SDK, need unmask the AF tuning function at tuning client
		(10)M865SW-801: Right side of Calibration tool window can't be seen
	enhancement:
		(1)M865SW-924: Release document should add the isp feature support
		(2)M865SW-820: Arbitrary resolution support from Camera sensor
		(3)M865SW-910: Remove python 2.x since ISP P18 package
		(4)M865SW-928: xml load failed should have log print
		(5)M865SW-912: tuning tool: can't preview on PC when capture format is YUV
		(6)M865SW-899: ISP release doc summary the methods of native HDR in camera porting guide
		(7)M865SW-886: Enable different scene evaluation modes
		(8)M865SW-926: ISP media server should report an error if not find calibration XML file
		(9)M865SW-741: Provide LSC map
		(10)M865SW-875: tuning tools,The linear mode and HDR modes need select menu
		(11)M865SW-874: improve user experience for tuning client
		(12)M865SW-929: Should not require or depend on unneeded XML file in SensorX_Entry.cfg
		(13)M865SW-749: Support setting sensitivity
		(14)BSLRM865-187: How to realize spot exposure mode

	known issue:
		(1)BSLRM865-211:basler sensor set fps have yellow mask
		(2)BSLRM865-209:Basler sensor with mode 2/3 have no effect about BLS when test  tuning tool and vvext feature.
		(3)BSLRM865-163:Basler/ov2775/oa08a20 sensor, the video jitter on 4k screen.
		(4)gstreamer test, lost frames. (due to nxp¡¯s gstreamer bug)
		(5)Camera stress test, fail to hear voice.
		(6)M865SW-621: on dual ISP camera case, first camera can't work if second camera module not loaded
		(7)dual basler mode3+mode1, the video is abnormal flickering Occasionally
		(8)OS08A20, AWB is not correct
		(9)Basler,dual_basler,dual_basler_ov2775, There may be exceptions in the bright light.
		(10)Basler, change mode on tuning-client,video Occasionally become abnormal.
		(11)Basler, mode0/2/3, Set setpoint, facing the special environment such as lights, the video will flash.
		(12)Android_CTS test, dual ov2775 fail 39 cases, dual basler fail 35 cases. please check test report for the details.
		(13) BSLRM865-188:fps control not so accuracy to pass test Edge Mode Control
		(14)The objective and subjective quality of the image is not very good, please refer to the test report
		(15) Ar1335 sensor,4096*3071, Occasionally purple frame.
		(16)BSLRM865-188:fps control not so accuracy to pass testEdgeModeControl
		(17)BSLRM865-140:Provide noise profile and green split of basler sensor
		(18)ov2775,08a20, HDR mode, video_test capture yuv, the first frame is layered

NXP_V5.15_P19:
        Bugfix:
               (1)M865SW-984: ISP Camera: P18 release vvext test system hung up
               (2)M865SW-982: dual basler: isp_media_server instance is missing: after dual basler camera modules load/unload stress test
               (3)M865SW-960: V4L2_ASYNC_MATCH_DEVNAME breaks Android GKI
               (4)M865SW-862: can not close AE completely
               (5)M865SW-773: build break due to no define for V4L2_ASYNC_MATCH_DEVNAME on Linux 5.15
               (6)M865SW-880: Hundreds camera CTS tests failed due to deque buffer blocked on 1080p
               (7)M865SW-970: Many camera CTS tests failed on isp-4.2.2-p18
        Enhancement
               (1)M865SW-977: ISP Camera: The no use files should delete after the P18 release
               (2)M865SW-979: ISP_DRIVER: patch for the ISP multi-power domain change for the 5.16 kernel power blk clt framework
               (3)M865SW-985: ISP Camera: dwe_isr.c build error found by gcc12
               (4)M865SW-967: ISP Camera: NXP's patches for VSI P19 release
               (5)M865SW-934: Calculate latency for processing image image ISP
               (6)M865SW-956: how to tune de-mosaic
               (7)M865SW-965: isp driver: VSI p18 release package need delet vvcam/common/vvnative.c
               (8)M865SW-922: Description of pipeline configuration for monochrome sensor
               (9)M865SW-978: loops for dual cameras to open/capture/close
               (10)M865SW-908: _AdaptOpenV4l2 change flow
               (11)M865SW-980: patches for os08a20 on Android
               (12)M865SW-717: Investigate to use C++ Boost library from Yocto
               (13)M865SW-914: ISP Camera : Native HDR curve define should be added in the document
               (14)M865SW-913: Resolve the build warning for Boost
               (15)M865SW-900: ov2775 HDR mode image color error
               (16)M865SW-937: ISP Camera: The log information should effectively provide the error reason, use error codes to indicate the cause of the specific error
        Known issue
          [Basler&nxp issue]
               (1)BSLRM865-211:basler sensor set fps have yellow mask
               (2)BSLRM865-209:Basler sensor with mode 2/3 have abnormal effect about BLS when test  tuning tool and vvext feature.
               (3)BSLRM865-163:Basler/ov2775/oa08a20 sensor, the video jitter on 4k screen.
               (4)Gstreamer test, lost frames. (due to nxp’s gstreamer bug)
               (5)Camera stress test, fail to hear voice.
               (6)M865SW-621: on dual ISP camera case, first camera can't work if second camera module not loaded
               (7)OS08A20, AWB is not correct.
               (8)Basler, dual_basler,dual_basler_ov2775, There may be exceptions in the bright light.
               (9)Basler, gstreamer test with mode0，occasional blue frames. Tuning with mode2/3，AE reset，video will be abnormal
               (10)Basler，Mode2/3，The picture flashes; disable AE and then adjust ec, the screen may be stuck, continue to adjust and recover
          [VSI issue]
               (1)Android_CTS test, dual ov2775 fail to run case, dual basler fail 35 cases，dual os8a202 fail 28. please check test report for the details.
               (2)The objective and subjective quality of the image is not very good, please refer to the test report
               (3)AR1335,mode0，color cast. AEC: error, AecSetHdrRatio: IsiSetHdrRatioIss failed
               (4)AR1335, vvext/tuning test, mode2,enable LSC, the video will have abnormal streaks.
               (5)AR1335, gstreamer test, mode2, video shakes and has purple frames
               (6)BSLRM865-188: fps control not so accuracy to pass testEdgeModeControl
               (7)BSLRM865-140: Provide noise profile and green split of basler sensor
               (8)Ov2775, tuning xml is stored in a non-existent path but it can be successful
               (9)Ov2775+mode1 and basler+mode2,the flashlight of the mobile phone is facing the lens, and the highlights overflow in the first few frames of the preview.
               (10)Os08a20，vvext, mode1/3, set fps, the video has obvious brightness changes.
               (11)Os08a20, mode0,tuning,enable LSC, the video will have abnormal streaks.

linux
	tuning compile:
		cd appshell
		mkdir build
		cd build
		cmake -DCMAKE_BUILD_TYPE=debug -DISP_VERSION=ISP8000NANO_V1802 -DPLATFORM=ARM64 -DAPPMODE=NATIVE -DQTLESS=1 -DFULL_SRC_COMPILE=1 -DWITH_DWE=1 -DWITH_DRM=1 -DSUBDEV_V4L2=1 -DSERVER_LESS=0  -DENABLE_IRQ=0 -DSDKTARGETSYSROOT=$SDKTARGETSYSROOT -Wno-dev  -DIMX_G2D=ON ..
		make -j4
	driver for tuning compile:
		cd vvcam/v4l2
		make VERSION=ISP8000NANO_V1802 KERNEL_SRC=xxx/build_v8/ ENABLE_IRQ=no
		
	isp_media_server compile:
		cd appshell
		mkdir build
		cd build
		make -DCMAKE_BUILD_TYPE=release -DISP_VERSION=ISP8000NANO_V1802 -DPLATFORM=ARM64 -DAPPMODE=V4L2 -DQTLESS=1 -DFULL_SRC_COMPILE=1 -DWITH_DWE=1 -DWITH_DRM=1 -DSERVER_LESS=1 -DSUBDEV_V4L2=1 -DENABLE_IRQ=1 -DSDKTARGETSYSROOT=$SDKTARGETSYSROOT -Wno-dev -DIMX_G2D=ON ..
		make -j4
	driver for isp_media_server:
		make VERSION=ISP8000NANO_V1802 KERNEL_SRC=xxx/build_v8/ ENABLE_IRQ=yes
		
android:
	isp_media_server compile:
		"copy all release packets files to xxx/vendor/nxp-private/verisilicon_sw_isp"
		cd appshell
		mkdir build-android
		cp android-build.sh build-android/
		cd build-android
		./android-build.sh
	driver for android compile:
		"copy vvcam to xxx/vendor/nxp-opensource/verisilicon_sw_isp_vvcam"
		./imx-make.sh vvcam -j8
		./imx-make.sh vendorbootimage -j8

