# PBD challenge

[Challenge accepted](https://matthias-research.github.io/pages/challenges/challenges.html)

## I've decided to improve speed of PBD simulation

I've done so by writing small pbd library in c++ and compiling it for web assembly (WASM). Results are "dosta dobri." Implementation in javascript is running on my pc with ~10fps. WASM implementation is running on my pc with ~60fps.


## You can test for yourself:

* [Js implementation](https://matthias-research.github.io/pages/challenges/bodyChainPBD.html)
* [Wasm implementation](https://ricko.us.to/PBDChallange)

## How to build

To build it, you will have to have emcc and you will have to edit cmake file (You have to set a location for emcc and glm and maybe something else) (I can't be bothered.) (Make a pull request if you want to edit cmake file.)





