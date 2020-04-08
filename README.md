# dubins

The dubins library is a C++ implementation of Dubins' paths, using C++17 standard. It offers extra features as openCV visualization (this is why you should install our demlo2d package first), and a vector type suitable for vq3 computation.

# Support

dubins is a result of the <a href="http://interreg-grone.eu">GRONE project</a>, supported by the Interreg "Grande Région" program of the European Union's European Regional Development Fund.


# Unix Installation

First, get the files.

``` 
git clone https://github.com/HerveFrezza-Buet/dubins
``` 

Then, you can install the package as follows. 

```
mkdir -p dubins/build
cd dubins/build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
cd ../..
```

The documentation is in /usr/share/doc/dubins
