 #!/bin/bash
export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0

gst-launch-1.0 -e \
  v4l2src device="/dev/video2" \
  ! image/jpeg,width=640,height=480 \
  ! jpegparse ! jpegdec ! videoconvert \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! qtivtransform \
  ! "video/x-raw,format=NV12,width=640,height=480" \
  ! tee name=split \
  split. ! queue max-size-buffers=20 leaky=no \
    ! videoconvert \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! videomixer name=mix \
    ! queue ! waylandsink fullscreen=false sync=false \
  split. ! queue max-size-buffers=2 leaky=downstream \
    ! qtimlvconverter \
    ! qtimltflite delegate=external \
      external-delegate-path=libQnnTFLiteDelegate.so \
      external-delegate-options="QNNExternalDelegate,backend_type=htp;" \
      model=yolov8_det.tflite \
    ! qtimlpostprocess \
      module=yolov8 \
      labels=labels.txt \
      results=10 \
      bbox-stabilization=true \
    ! "video/x-raw,format=BGRA,width=640,height=480" \
    ! queue max-size-buffers=2 leaky=downstream \
    ! mix.