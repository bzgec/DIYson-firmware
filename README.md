# DIYson firmware

This is a firmware for [DIYson](https://github.com/stevenbennett/DIYson) from [Steven Bennett](https://github.com/stevenbennett) ([YouTube playlist - DIYson Lamp Build Log](https://www.youtube.com/playlist?list=PL0szyq6FLzZi2BB8-iA09LrTv0rd4K04u)).

You probably need to configure touch filtering and change pins - [DIYson.ino](./DIYson.ino).

Tested on [Seeed Studio XIAO SAMD21](https://wiki.seeedstudio.com/Seeeduino-XIAO/).

Note that [Seeed Studio XIAO SAMD21](https://wiki.seeedstudio.com/Seeeduino-XIAO/) and [Adafruit QT Py - SAMD21](https://www.adafruit.com/qtpy) boards are not really pin compatible:
- DAC pins:
  | Pin  | XIAO | QT Py |
  | ---- | ---: | ----: |
  | `A0` |    x |     x |
  | `A1` |      |     x |
  | `A2` |      |     x |

- Touch pins:
  | Pin       | XIAO | QT Py |
  | --------- | ---: | ----: |
  | `A0`      |    x |     x |
  | `A1`      |    x |     x |
  | `A2`      |      |     x |
  | `A3`      |      |     x |
  | `A6`/`TX` |    x |     x |
  | `A7`/`RX` |    x |     x |
  | `A8`      |    x |       |
  | `A9`      |    x |       |
  | `A10`     |    x |       |


## TODO
- [ ] LED fading
