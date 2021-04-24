import time


def time_it(f):
    def timed(*args, **kw):
        ts = time.time()
        result = f(*args, **kw)
        te = time.time()
        print(f'{f.__name__} took: {te-ts} sec')
        return result
    return timed
