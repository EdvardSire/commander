import matplotlib.pyplot as plt
from shapelysmooth import taubin_smooth, chaikin_smooth, catmull_rom_smooth
from shapely import LineString

def main():
    gem = LineString([(0,0), (1,5), (-10,-6)])
    plt.plot(*gem.xy)

    for smoother in [taubin_smooth, chaikin_smooth, catmull_rom_smooth]:
        plt.plot(*(smoother(gem)).xy, label=smoother.__name__)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
