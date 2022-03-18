# PIL-Firmware

## Installation and Build Instructions

### GNU Make

```  
    cd /path/to/project
    git checkout develop

    git submodule update --init --recursive --remote
    cd WLoopCAN
    make pod_indicator_light

    cd ..
    make 
```