import sys
import time
import numpy as np
import os

initData = {
    'forceFalloff': 1.0,
    'targetForce': 0.3,
    'avoidRange': 0.7,
    'avoidForce': 1.5,
    'maxLength': 0.1,
}

if __name__ == '__main__':
    print(type(initData))
    print(initData['forceFalloff'])
    print(type(initData['forceFalloff']))
