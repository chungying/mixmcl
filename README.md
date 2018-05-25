# Mixmcl
2D MCL with mixture proposal distributions

## Requirement
### Nuklei
This library is used to perform kernel density estimation and kdtree sampling.
```
$ sudo apt-get install gcc g++ make pkg-config python libgsl0-dev libblas-dev liblapack-dev libboost-all-dev
$ git clone https://github.com/chungying/nuklei.git
$ cd nuklei
$ ./scons.py
$ sudo ./scons.py install
```

### FLANN
This library is for kdtree and 1-nearest-neighbor search.
```
$ sudo apt-get install -y libflann-dev
```

### ROS packages
Most packages could be installed via rosdep or apt-get. Following two packages should be installed as well.
* [amcl_modified](https://github.com/chungying/amcl_modified.git)
* [stamped_std_msgs](https://github.com/chungying/stamped_std_msgs)

## Functions
* MCL with mixture proposals by using pre-built sampling
* MCL with mixture proposals by using Evolutionary Markov Chain Monte Carlo sampling

### Performance comparison
#### Global Localisation Problem
Some results for AMCL, MCL, MIXMCL, and MCMCL (the proposed method) are shown here.

mp: maximum particle number
ri: resample intervel

[![Output sample](https://j.gifs.com/APYGyj.gif)](https://youtu.be/dVkXle5kVsw)
[![Output sample](https://j.gifs.com/pQjARp.gif)](https://youtu.be/-mXXptsgW4I)
[![Output sample](https://j.gifs.com/E9VkZY.gif)](https://youtu.be/soJFdU9mu3Y)
[![Output sample](https://j.gifs.com/KZVqYM.gif)](https://youtu.be/RvT5cKMrz3o)

#### Kidnapped Robot Problem



