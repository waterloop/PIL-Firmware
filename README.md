# PIL-Firmware

## Installation and Build Intructions

### GNU Make

```  
    cd /path/to/project
    git checkout develop

    git submodule --init --recursive --remote
    cd WLoopCAN
    make pod_indicator_light

    cd ..
    make 
```