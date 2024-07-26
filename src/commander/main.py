import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splev, splprep
from shapely import LineString
from shapelysmooth import catmull_rom_smooth, chaikin_smooth, taubin_smooth


def basis_spline(geometry: LineString, smoothing_condition=20):
    waypoints = np.array([[x, y] for x, y in zip(*geometry.xy)])
    tck, _ = splprep([waypoints[:, 0], waypoints[:, 1]], s=smoothing_condition)
    u_fine = np.linspace(0, 1, len(waypoints) * 2)
    x_smooth, y_smooth = splev(u_fine, tck)
    return LineString(np.c_[x_smooth, y_smooth])


def main():
    # waypoints = np.array([[0, 0], [1, 2], [2, 1], [3, 3], [4, 2], [5, 5]])
    waypoints = np.array([[104.17562103271484, 731.805908203125], [119.1114414477948, 732.7505108203749], [137.82936678248709, 731.9152169935443], [147.7661882198937, 732.6463308683597], [167.6495843381497, 734.1353374697641], [179.5230227778828, 731.9034421950587], [187.37414777409685, 730.1146318843042], [192.2779760004238, 729.1386868414462], [199.36263104865387, 725.7091153243933], [205.6106145627725, 725.7489936323408], [210.54084133859777, 725.1615708052584], [213.27644031699873, 723.9463176608328], [214.472693724914, 722.0727948644982], [217.83655389791102, 718.3735456519519], [218.78091905089533, 716.7987245630268], [223.59906392872392, 718.4026542816777], [232.96202699133005, 717.0759440153683], [237.87162221562278, 718.0224506099581], [247.51836461643796, 720.5687811411001], [256.206912431769, 725.3645233931657], [265.76741888637804, 727.8777011378479], [274.76063143951376, 727.3271774780718], [284.2577432741642, 729.6278668967185], [289.1492447602721, 730.6638269400621], [294.1399140905201, 730.3585080311782], [299.1305834207681, 730.0531891222943], [299.119510781278, 730.8097472109852], [300.0, 730.0]])  # fmt: skip
    geometry = LineString(waypoints)
    basis_spline(geometry)
    plt.plot(*geometry.xy)

    for smoother in [taubin_smooth, chaikin_smooth, catmull_rom_smooth, basis_spline]:
        plt.plot(*(smoother(geometry)).xy, label=smoother.__name__)
    plt.legend()
    plt.grid(True)
    # plt.gca().set_aspect("equal")
    plt.show()


if __name__ == "__main__":
    main()
