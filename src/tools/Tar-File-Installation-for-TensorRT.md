### [Tar File Installation for TensorRT](https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html#installing-tar)

This section contains instructions for installing TensorRT from a tar file. 

Note: Before issuing the following commands, you'll need to replace  7.x.x.x with your specific TensorRT version. The following commands are examples. 

1. Install the following dependencies, if not already present:
   - [10.0](https://docs.nvidia.com/cuda/archive/10.0/index.html), or [10.2](https://docs.nvidia.com/cuda/archive/10.2/index.html)
   - [cuDNN 7.6.5](https://docs.nvidia.com/deeplearning/sdk/cudnn-release-notes/rel_765.html#rel_765)
   - Python 2 or Python 3 (Optional) 

2. [Download](https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html#downloading) the TensorRT tar file  that matches the Linux distribution you are using.

3. Choose where you want to install TensorRT. This tar file will  install everything into a subdirectory called TensorRT-7.x.x.x.

4. Unpack the tar file.

   ```
   version=”7.x.x.x”
   os=”<os>”
   arch=$(uname -m)
   cuda=”cuda-x.x”
   cudnn=”cudnn7.x”
   tar xzvf TensorRT-${version}.${os}.${arch}-gnu.${cuda}.${cudnn}.tar.gz
   ```

​       **Where: **

   - 7.x.x.x is your TensorRT version                                 

   - <os>is:                         
- Ubuntu-16.04
      - Ubuntu-18.04
- CentOS-7.6
     
   - cuda-x.x is CUDA version 10.0, or 10.2.                                 

   - cudnn7.x is cuDNN version 7.6. This directory will have sub-directories like lib ,                              include , data,  etc…

   ```
ls TensorRT-${version}
bin  data  doc  graphsurgeon  include  lib  python  samples  targets  TensorRT-Release-Notes.pdf  uff
   ```

5. Add the absolute path to the TensorRTlib directory to the environment variable                                 LD_LIBRARY_PATH:

   ```
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<TensorRT-${version}/lib>
   ```

6. Install the Python TensorRT wheel file.

   ```
   cd TensorRT-${version}/python
   ```

   If using Python  2.7:

   ```
sudo pip2 install tensorrt-*-cp27-none-linux_x86_64.whl
   ```
   
   If using Python 3.x:

   ```
sudo pip3 install tensorrt-*-cp3x-none-linux_x86_64.whl
   ```

7. Install the Python UFF wheel file. This is only required if you plan to use TensorRT with TensorFlow.

   ```
   cd TensorRT-${version}/uff
   ```

   If using Python  2.7:

   ```
sudo pip2 install uff-0.6.5-py2.py3-none-any.whl
   ```
   
   If using Python 3.x:

   ```
sudo pip3 install uff-0.6.5-py2.py3-none-any.whl
   ```

   In either case, check the installation  with:
   
   ```
which convert-to-uff
   ```

8. Install the Python graphsurgeon wheel file.

   ```
   cd TensorRT-${version}/graphsurgeon
   ```

   If using Python 2.7:

   ```
sudo pip2 install graphsurgeon-0.4.1-py2.py3-none-any.whl
   ```
   
   If using Python 3.x:

   ```
sudo pip3 install graphsurgeon-0.4.1-py2.py3-none-any.whl
   ```

9. Verify the installation:

   1. Ensure that the installed files are located in the correct directories.                                       For example, run the tree -d command to check whether all supported installed files are in place in the lib, include, data, etc…  directories.
2. Build and run one of the shipped samples, for example, sampleMNIST in the installed directory. You should be able to compile and execute the sample without additional settings. For more information, see the [“Hello World” For TensorRT (sampleMNIST)](https://github.com/NVIDIA/TensorRT/tree/release/7.0/samples/opensource/sampleMNIST).
   3. The Python samples are in the samples/python directory.