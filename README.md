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

![Output sample](https://github.com/chungying/mixmcl/video/ex3/amcl_mp2000_ri1.mp4)

#### Kidnapped Robot Problem



