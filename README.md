```
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev 
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk2.0-dev
sudo apt-get install libatlas-base-dev gfortran

sudo apt install python3-opencv 

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update

sudo apt-get install libedgetpu1-std
sudo apt-get install python3-pycoral  --yes

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-tflite-runtime
pip3 install imutils



cd Desktop
git clone https://github.com/cuong3004/Trash_Cuong

cd PCD8544
bash setup.py

cd 

cd build-LCDGui-Desktop-Release
sudo ./LCDGui
```

```
Chú ý

Cần phải sửa lại đường dẫn trong LCDGui (đã được commend)
build-LCDGui-Desktop-Release được build trên một máy khác (máy open robothus)
```