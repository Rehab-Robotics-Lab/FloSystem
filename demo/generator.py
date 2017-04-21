import random
import json

mouth = [[1 if random.random() > .5 else 0 for e in range(16)] for i in range(8)]
eye = [[1 if random.random() > .5 else 0 for e in range(8)] for i in range(8)]

