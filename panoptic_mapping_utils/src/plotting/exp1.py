import collections
import os
from matplotlib import pyplot as plt

# Params
keys = [
    'Map Size [MB]', 'Average Reconstruction Error [cm]',
    'Reconstruction Error RMSE [cm]', 'Coverage [%]'
]  # x, y
output_dir = '/home/lukas/Pictures/pan'
output_name = 'multi_res'
store_output = True
plot_error = True

# name: [mean absolute distance [cm], map size [MB], RMSE, unknown points
# (of 3201033)
data = {
    'supereight multires 10cm': [3.899, 8.9, 6.277, 353298],
    'supereight multires 5cm': [2.051, 37.4, 3.212, 517256],
    'supereight multires 2cm': [0.962, 226.8, 1.286, 733611],
    'supereight multires 1cm': [0.657, 899.1, 0.793, 832092],
    'supereight dense 10cm': [3.90, 3.6, 6.276, 353360],
    'supereight dense 5cm': [2.051, 22.5, 3.218, 517316],
    'supereight dense 2cm': [0.961, 136.3, 1.284, 734710],
    'supereight dense 1cm': [0.663, 540.2, 0.797, 847184],
    # 'voxblox 20cm': [7.076, 0.8, 9.911, 371626],
    'voxblox 10cm': [3.051, 3.7, 4.609, 578613],
    'voxblox 5cm': [1.505, 21.0, 2.258, 744807],
    'voxblox 2cm': [0.753, 236.3, 1.040, 933362],
    # 'voxblox_uni 20cm': [10.664, 0.8, 14.210, 336338],
    # 'voxblox_uni 10cm': [4.395, 3.7, 6.376, 546224],
    # 'voxblox_uni 5cm': [2.098, 21.1, 3.068, 716764],
    # 'voxblox_uni 2cm': [0.903, 237.2, 1.238, 902464],
    # 'panmap single_tsdf 20cm': [6.194, 0.8, 10.659, 665660],
    # 'panmap single_tsdf 10cm': [2.792, 5.0, 4.945, 707937],
    # 'panmap single_tsdf 5cm': [1.417, 29.2, 2.440, 888050],
    # 'panmap multi GT tiny': [1.0065, 17.1, 1.650, 1063186],
    'panmap multi GT small': [1.103, 9.7, 1.775, 985201],
    'panmap multi GT medium': [1.597, 4.7, 2.604, 879226],
    'panmap multi GT large': [2.383, 3.8, 4.198, 769236],
    'panmap multi Detectron small': [1.485, 15.7, 2.511, 914748],
    'panmap multi Detectron medium': [2.169, 7.9, 3.764, 825339],
    'panmap multi Detectron large': [3.120, 5.0, 5.447, 722525],
}

# Plot
plt.rcParams.update({'font.size': 12})
keylist = sorted(data.keys())
styles = {
    # 'panmap single_tsdf ': 'xb',
    'supereight multires': 'ok',
    'supereight dense': 'xk',
    'voxblox ': 'og',
    # 'voxblox_uni ': 'xg',
    'panmap multi GT': 'ob',
    'panmap multi Detectron': 'xb'
}
styles = collections.OrderedDict(sorted(styles.items()))
for k in keylist:
    for s in styles.keys():
        if k.startswith(s):
            style = styles[s]
            text = k[len(s):]
            break
    x = data[k][1]
    y = data[k][0]  # 0 - MAD, 2 - ERROR
    if not plot_error:
        y = (3201033.0 - data[k][2]) / 32010.33
    plt.scatter(x, y, c=style[1], marker=style[0])
    # plt.annotate(text, (x, y))

# Axes
plt.semilogx()
# plt.semilogy()
plt.xlabel(keys[0])
if plot_error:
    plt.ylabel(keys[1])
    plt.ylim((0, 4))
else:
    plt.ylabel(keys[2])
    plt.ylim((60, 100))

# Legend
artists = []
labels = [['Voxblox', 'og'], ['Supereight (multires)', 'ok'],
          ['Supereight (dense)', 'xk'], ['Ours (ground truth)', 'ob'],
          ['Ours (detectron)', 'xb]']]
# labels = sorted(labels, reverse=True)
for l in labels:
    artists.append(plt.scatter(0, 0, c=l[1][1], marker=l[1][0]))
plt.legend(artists, [l[0] for l in labels])

# Save
plt.gcf().set_size_inches(6, 3.7, forward=True)
plt.tight_layout()
if plot_error:
    output_name += "_error.jpg"
else:
    output_name += "_coverage.jpg"
if store_output:
    plt.savefig(os.path.join(output_dir, output_name), dpi=300)
plt.show()
