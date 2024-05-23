# Tracker : The Embedded Camera Stand which Tracking Target Person using YOLO DeepSORT

## ✨Summary
이 repository는 YOLO DeepSORT 기반의 Target Person Tracking 기능을 제공하는 Embedded 카메라 스탠드의 소스 코드입니다.
<br>
<br>
Operation Video
<br>
https://youtu.be/oVTTcawM6HI?si=1EZC0dn0Wkzd7s7u

<br>

<div style="text-align: center;">
  
  <img src="https://github.com/cobang0111/Tracker-Object_Tracking_Camera_Stand/assets/97373900/988a335b-6c52-473e-b67d-9b881aeff83e" width="480">
  
</div>

<br>

Jetson Nano 4GB, CUDA 10.2, JetPack 4.6, Python 3.6, PyTorch v1.10.0 - torchvision v.0.11.1, YOLOv7 에서 테스트 되었으며, USB webcam (640x480) 실시간 처리를 기반으로 크게 다음과 같은 2가지 작동 모드를 제공합니다.

<br>

- Fast Operation Mode (17fps) : 화면 내 가장 크게 감지된 사람을 추적합니다.
- Accurate Operation Mode (3fps) : DeepSORT 를 사용하여 ID 기반 추적을 진행합니다.

<br>
테스트 간에 아래와 같은 한계점이 존재했습니다. 
<br>

이에 대해서는 개선이 필요합니다.

- Jetson Nano 4GB 의 Memory 부족 현상
- TRT 기반의 DeepSORT 구현 필요
- 2축 Servo Motor를 기반으로 한 스탠드를 사용하여, Position 제어가 불가능.
  이로 인해 DeepSORT 과정에서 Kalman filter 와 Hungarian Algorithm 을 기반으로 추적을 할 때 프레임 간의 물체의 pixel 위치가 많이 달라져 추적 대상을 놓치는 경우가 발생합니다.

<br>

## ✨Prerequisite

Jetson Nano 에서 TRT 기반의 분석이 가능하도록 사전 환경 설정이 필요합니다.

<br>

Jetson Nano 의 사양을 최고 사양으로 설정하고, Swap memory 확장이 필요합니다.

<br>

opencv가 CUDA 를 사용하도록 cmake build를 수행해야 합니다.

<br>
<br>


## ✨Install
예시로 /workspace 디렉토리에서 이 repository를 clonning 하겠습니다.
```bash
cd /workspace

git clone https://github.com/cobang0111/Tracker-Object_Tracking_Camera_Stand.git

cd Tracker-Object_Tracking_Camera_Stand
```

<br>

## ✨Execution

Mode 1 - Fast Operation Mode (17fps) : 화면 내 가장 크게 감지된 사람을 추적
```bash

python3 yolov7_trt_no_deepsort_xy.py

```

<br>


Mode 2 - Accurate Operation Mode (3fps) : DeepSORT 를 사용하여 ID 기반 추적
```bash

cd yolov7

python3 yolov7_pt_sort_motor.py --weights yolov7-tiny.pt --no-trace --view-img --nosave --source 0 --show-fps --seed 2 --track --classes 0 --show-track

```


<br>

## ✨Result

![image](https://github.com/cobang0111/Tracker-Object_Tracking_Camera_Stand/assets/97373900/2d7773ac-d84b-4949-9ce6-39bb30f31062)


<br>

## ✨Reference
All right and license follow the official yolov7 and below reference repository

https://github.com/WongKinYiu/yolov7

https://github.com/spehj/jetson-nano-yolov7-tensorrt

https://github.com/haroonshakeel/yolov7-object-tracking


<br>

<br>



