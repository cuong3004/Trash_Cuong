```bash

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update

sudo apt-get install libedgetpu1-std

sudo apt-get install python3-pycoral  --yes

pip3 show tflite_runtime

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-tflite-runtime

mkdir google-coral && cd google-coral
git clone https://github.com/google-coral/examples-camera --depth 1

cd examples-camera
sh download_models.sh

cd opencv
bash install_requirements.sh

pip3 install pigpio
pip3 install imutils

echo 'sudo pigpiod' >> ~/.bashrc
source ~/.bashrc

cd 
git clone https://github.com/nRF24/RF24.git
cd RF24/
./configure 
sudo make install
cd pyRF24/
sudo apt-get install python3-dev libboost-python-dev 
sudo apt-get install python3-setuptools 
sudo python3 setup.py build
sudo python3 setup.py install

cd ~/Desktop
git clone https://github.com/cuong3004/Trash_Cuong

cd Trash_Cuong/PCD8544
bash setup.py

cd ..
cp . ../
cd ..

cd build-LCDGui-Desktop-Release
sudo ./LCDGui
```