import os
from collections import defaultdict

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.style
import plotly.express as px
import sklearn.preprocessing
from cycler import cycler
from scipy.interpolate import interp1d
from scipy.optimize import brentq
from sklearn.metrics import confusion_matrix
from sklearn.metrics import roc_curve, auc

from transforms import *

os.environ["KMP_DUPLICATE_LIB_OK"] = 'True'
mpl.style.use('classic')

SEED = 1
BATCH_SIZE = 256
EPOCHS = 100
LR = 0.001
NUM_MEASUREMENT = 128
TRUNCATE2, TRUNCATE5 = 128, 512
CHANNELS = [2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5745, 5765, 5785, 5805, 5825, 5845]
FEATURES = ['noise', 'max_magnitude', 'total_gain_db', 'base_pwr_db', 'rssi', 'relpwr_db', 'avgpwr_db']
STAT_FEATURES = ['mean', 'std', 'min', '25%', '50%', '75%', 'max', 'mad']
LABELS = {'floor': 0, 'inter_mid': 1, 'inter_high': 2, 'jamming': 3}


class Scheduler:
    """Updates optimizer's learning rates using provided scheduling function."""

    def __init__(self, opt, schedule):
        self.opt = opt
        self.schedule = schedule
        self.history = defaultdict(list)
        self.last_lr = 0

    def step(self, t):
        for i, group in enumerate(self.opt.param_groups):
            lr = self.opt.defaults['lr'] * self.schedule(t)
            group['lr'] = lr
            self.history[i].append(lr)
            self.last_lr = lr


def model_summary(model):
    print(model)
    print(f'Model size: {sum([param.nelement() for param in model.parameters()]) / 1000000} (M)')


def init_center_c(loader, net, device, eps=0.1):
    n_samples = 0
    c = torch.zeros(net.rep_dim, device=device)

    net.eval()
    with torch.no_grad():
        for inputs, _ in loader:
            inputs = inputs.to(device)
            outputs = net.encode(inputs)
            n_samples += outputs.shape[0]
            c += torch.sum(outputs, dim=0)

    c /= n_samples

    c[(abs(c) < eps) & (c < 0)] = -eps
    c[(abs(c) < eps) & (c > 0)] = eps
    print(f'Computed center: {c}')
    return c


def roc(labels, scores, plot=True):
    fpr = dict()
    tpr = dict()

    # True/False Positive Rates.
    fpr, tpr, thresholds = roc_curve(labels, scores)
    auroc = auc(fpr, tpr)

    # new_auc = auroc_score(labels, scores)

    # Equal Error Rate
    eer = brentq(lambda x: 1. - x - interp1d(fpr, tpr)(x), 0., 1.)

    if plot:
        # Colors, color cycles, and colormaps
        mpl.rcParams['axes.prop_cycle'] = cycler(color='bgrcmyk')

        # Colormap
        mpl.rcParams['image.cmap'] = 'jet'

        # Grid lines
        mpl.rcParams['grid.color'] = 'k'
        mpl.rcParams['grid.linestyle'] = ':'
        mpl.rcParams['grid.linewidth'] = 0.5

        # Figure size, font size, and screen dpi
        mpl.rcParams['figure.figsize'] = [8.0, 6.0]
        mpl.rcParams['figure.dpi'] = 80
        mpl.rcParams['savefig.dpi'] = 100
        mpl.rcParams['font.size'] = 12
        mpl.rcParams['legend.fontsize'] = 'large'
        mpl.rcParams['figure.titlesize'] = 'medium'

        # Marker size for scatter plot
        mpl.rcParams['lines.markersize'] = 3

        # Plot
        mpl.rcParams['lines.linewidth'] = 0.9
        mpl.rcParams['lines.dashed_pattern'] = [6, 6]
        mpl.rcParams['lines.dashdot_pattern'] = [3, 5, 1, 5]
        mpl.rcParams['lines.dotted_pattern'] = [1, 3]
        mpl.rcParams['lines.scale_dashes'] = False

        # Error bar
        mpl.rcParams['errorbar.capsize'] = 3

        # Patch edges and color
        mpl.rcParams['patch.force_edgecolor'] = True
        mpl.rcParams['patch.facecolor'] = 'b'

        plt.figure()
        lw = 1
        plt.plot(fpr, tpr, color='darkorange', label='(AUC = %0.4f, EER = %0.4f)' % (auroc, eer))
        plt.plot([eer], [1 - eer], marker='o', markersize=3, color="navy")
        # plt.plot([0, 1], [1, 0], color='navy', lw=1, linestyle=':')
        plt.plot([0, 0], [0, 1], color='navy', linestyle=':')
        plt.plot([0, 1], [1, 1], color='navy', linestyle=':')
        plt.xlim([-0.05, 1.0])
        plt.ylim([0.0, 1.05])
        plt.xlabel('False Positive Rate')
        plt.ylabel('True Positive Rate')
        # plt.title('Receiver operating characteristic')
        plt.legend(loc="lower right")
        # plt.savefig(f'{args.exp_path}/auc_{epoch}.svg', bbox_inches='tight', format='svg', dpi=800)
        plt.show()
        plt.close()

    return auroc


def init_weights(net, init_type='normal', gain=0.02):
    def init_func(m):
        with torch.no_grad():
            classname = m.__class__.__name__
            if hasattr(m, 'weight') and (classname.find('Conv') != -1 or classname.find('Linear') != -1):
                if init_type == 'normal':
                    torch.nn.init.normal_(m.weight.data, mean=0.0, std=gain)
                elif init_type == 'xavier':
                    torch.nn.init.xavier_normal_(m.weight.data, gain=gain)
                elif init_type == 'kaiming':
                    torch.nn.init.kaiming_normal_(m.weight.data, a=0, mode='fan_in')
                elif init_type == 'orthogonal':
                    torch.nn.init.orthogonal_(m.weight.data, gain=gain)
                else:
                    raise NotImplementedError('initialization method [%s] is not implemented' % init_type)
                if hasattr(m, 'bias') and m.bias is not None:
                    torch.nn.init.constant_(m.bias.data, 0.0)
            elif classname.find('BatchNorm2d') != -1:
                torch.nn.init.normal_(m.weight.data, 1.0, gain)
                torch.nn.init.constant_(m.bias.data, 0.0)

    net.apply(init_func)


def scores_landscape(net, train_samples, c, min_input=-10, max_input=10, size=512):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    inputs = np.zeros((size, size, 2)).astype(np.float32)
    step = (max_input - min_input) / size
    for i in range(size):
        for j in range(size):
            inputs[i][j][0] = min_input + (step * i) + (step / 2)
            inputs[i][j][1] = min_input + (step * j) + (step / 2)

    inputs = torch.from_numpy(inputs).view(-1, 2)
    scores = torch.zeros(size * size).view(-1)
    net.eval()
    with torch.no_grad():
        for i in range(0, inputs.shape[0] // 64):
            input = inputs[i * 64:i * 64 + 64]
            input = input.to(device)
            outputs = net.encode(input)
            dist = torch.sum((outputs - c) ** 2, dim=1)
            scores[i * 64: i * 64 + 64] = dist

    scores = scores.view(size, size).numpy()

    # train_samples = train_samples.numpy()
    # plt.scatter(train_samples[:, 0], train_samples[:, 1], color='blue')
    plt.imshow(scores.T, cmap='gray', interpolation='bilinear', origin='lower')
    plt.tight_layout()
    plt.colorbar()
    plt.show()


def plot(samples, targets, file, dataset):
    n_unlab, n_norm, n_anom = 0, 0, 0
    for i in range(targets.shape[0]):
        if targets[i] == -1:
            n_anom += 1
        elif targets[i] == 0:
            n_unlab += 1
        else:
            n_norm += 1

    cdict = {-1: 'red', 0: 'grey', 1: 'blue'}
    fig, ax = plt.subplots(figsize=(8, 6), dpi=300)
    ax.set_title(f"# unlabeled: {n_unlab}, # normal: {n_norm}, # anomaly: {n_anom}")

    for g in np.unique(targets):
        if g == -1:
            label = 'Anomaly'
        elif g == 0:
            label = 'Unlabeled'
        else:
            label = "Normal"

        ix = np.where(targets == g)
        ax.scatter(samples[ix, 0], samples[ix, 1], c=cdict[g], label=label, s=30, linewidth=0.4)
    ax.legend(loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.15))
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    plt.tight_layout()
    plt.savefig(f'results/{dataset}/{file}', dpi=300)
    # plt.show()


def scores_contour(net, c, test_samples, test_targets, auroc, dataset, add_points=False):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    x = np.linspace(-10, 10, 100)
    y = np.linspace(-10, 10, 100)
    zs = np.zeros((100, 100))

    net.eval()
    with torch.no_grad():
        for i in range(x.shape[0]):
            for j in range(y.shape[0]):
                # zs[i][j] = x[i] ** 2 + y[j] ** 2
                input = torch.tensor([x[i], y[j]], dtype=torch.float32, device=device)
                output = net.encode(input)
                dist = torch.sum((output - c) ** 2)
                zs[i][j] = dist.to('cpu').numpy().item()

    fig, ax = plt.subplots(1, 1, figsize=(8, 6), dpi=300)
    ax.set_title(f"Score's Contours Plot - AUROC: {auroc:.5f}")

    cp = ax.contourf(x, y, np.transpose(zs), 1000)
    cbar = fig.colorbar(cp)
    cbar.set_label('Score', rotation=270, labelpad=15)
    ax.autoscale(False)

    if add_points:
        cdict = {-1: 'red', 0: 'grey', 1: 'blue'}
        for g in np.unique(test_targets):
            label = 'Normal' if g == 1 else 'Anomaly'
            ix = np.where(test_targets == g)
            ax.scatter(test_samples[ix, 0], test_samples[ix, 1], c=cdict[g], label=label, s=30, linewidth=0.4)
        ax.legend(loc='lower center', ncol=2, bbox_to_anchor=(0.5, -0.15))

    plt.tight_layout()
    plt.savefig(f'results/{dataset}/scores_contour', dpi=300)
    # plt.show()


def plot_timeseries(series: np.ndarray, target: np.ndarray, label_encoder: sklearn.preprocessing.LabelEncoder, features, jam_frequency='2.4'):
    if jam_frequency == '2.4':
        str_labels = ['floor', 'inter_mid', 'inter_high', 'jam_2GHz_20cm_0dBm', 'jam_2GHz_40cm_0dBm',
                      'jam_2GHz_60cm_0dBm', 'simulated', 'crypto']
    else:
        str_labels = ['floor', 'inter_mid', 'inter_high', 'jam_5GHz_20cm_0dBm', 'jam_5GHz_40cm_0dBm',
                      'jam_5GHz_60cm_0dBm', 'simulated', 'crypto']

    permuted_indices = np.random.permutation(series.shape[0])
    series = series[permuted_indices]
    target = target[permuted_indices]

    subseries = []
    for str_label in str_labels:
        for j in range(len(target)):
            encoded_label = label_encoder.transform([str_label])
            if target[j] == encoded_label.item():
                subseries.append(series[j])
                break
    series = np.array(subseries)

    # Create a list of colors to use for each time series
    colors = px.colors.qualitative.G10

    # Create a figure with 6 subplots, one for each feature
    fig, ax = plt.subplots(series.shape[1], 1, figsize=(10, 30))

    # Loop through each of the 6 features
    for i in range(series.shape[1]):
        # Loop through each of the 5 time series
        for j in range(len(str_labels)):
            # Plot the jth time series of the ith feature with the color from the colors list
            ax[i].plot(series[j, i, :], color=colors[j], label=str_labels[j])
            ax[i].set_title(f'{features[i]}')
        ax[i].set_xlim([0, 127])

    # Show the plot
    plt.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.25))
    plt.tight_layout()
    # plt.show()
    plt.savefig('plots/plot2.pdf', dpi=300)


def plot_timeseries_augment(series: np.ndarray, target: np.ndarray, label_encoder: sklearn.preprocessing.LabelEncoder, jam_frequency='2.4'):
    if jam_frequency == '2.4':
        str_labels = ['floor', 'inter_mid', 'inter_high', 'jam_2GHz_20cm_0dBm', 'jam_2GHz_40cm_0dBm',
                      'jam_2GHz_60cm_0dBm']
    else:
        str_labels = ['floor', 'inter_mid', 'inter_high', 'jam_5GHz_20cm_0dBm', 'jam_5GHz_40cm_0dBm',
                      'jam_5GHz_60cm_0dBm']

    subseries = []
    for str_label in str_labels:
        for j in range(len(target)):
            encoded_label = label_encoder.transform([str_label])
            if target[j] == encoded_label.item():
                subseries.append(series[j])
                break
    series = np.array(subseries)

    # Define TSTensor and augment it
    augmented = TSTensor(series)

    # Crop
    resized_crop = TSRandomResizedCrop(scale=(0.05, 1.0))(augmented, split_idx=0)
    crop_pad = TSRandomCropPad(magnitude=0.5)(augmented, split_idx=0)

    # Noise
    time_noise = TSTimeNoise(magnitude=1.0)(resized_crop, split_idx=0)
    # gaussian = TSGaussianNoise(magnitude=1.0)(time_noise, split_idx=0)

    augmented = time_noise

    # Create a list of colors to use for each time series
    colors = px.colors.qualitative.G10

    # Define the features
    features = ['Max. Magnitude', 'Total Gain (dBm)', 'Base Power (dBm)', 'RSSI', 'Real Power (dBm)',
                'Average Power (dBm)']

    # Create a figure with 6 subplots, one for each feature
    fig, ax = plt.subplots(6, 2, figsize=(10, 15))

    # Loop through each of the 6 features
    for i in range(6):
        # Loop through each of the 5 time series
        for j in range(6):
            # Plot the jth time series of the ith feature with the color from the colors list
            ax[i][0].plot(series[j, i, :], color=colors[j], label=str_labels[j])
            ax[i][0].set_title(f'{features[i]}')
            ax0_xlim = ax[i][0].get_ylim()

            ax[i][1].plot(augmented[j, i, :], color=colors[j])
            ax[i][1].set_title(f'{features[i]}')
            ax[i][1].set_ylim(ax0_xlim)

        ax[i][0].set_xlim([0, 127])
        ax[i][1].set_xlim([0, 127])

    # Show the plot
    plt.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.25))
    plt.tight_layout()
    plt.show()
    # plt.savefig('plots/augmented.pdf', dpi=300)
    print(series.shape)
    quit()


def plot_timeserie_augment(serie: np.ndarray, augment='cut_out'):
    shape = serie.shape

    if augment == 'crop_resize':
        serie = serie.view(1, shape[0], shape[1])
        augmented = random_crop_resize(serie, scale=(0.1, 1.0), size=128).view(shape[0], shape[1])
    elif augment == 'gaussian':
        augmented = gaussian_noise(serie)
    elif augment == 'flip':
        augmented = horizontal_flip(serie)
    elif augment == 'window_warp':
        augmented = window_warp(serie)
    elif augment == 'time_warp':
        augmented = time_warp(serie)
    elif augment == 'cut_out':
        augmented = cut_out(serie)

    # Create a list of colors to use for each time series
    colors = px.colors.qualitative.G10

    # Define the features
    features = ['Max. Magnitude', 'Total Gain (dBm)', 'Base Power (dBm)', 'RSSI', 'Real Power (dBm)',
                'Average Power (dBm)']

    # Create a figure with 6 subplots, one for each feature
    n_rows = 6
    fig, ax = plt.subplots(n_rows, 2, figsize=(10, 15))

    # Loop through each of the 6 features
    for i in range(n_rows):
        # Plot the jth time series of the ith feature with the color from the colors list
        ax[i][0].plot(serie[i, :], color=colors[0], label='Original')
        ax[i][0].set_title(f'{features[i]}')
        ax0_xlim = ax[i][0].get_ylim()

        ax[i][1].plot(augmented[i, :], color=colors[1], label='Augmented')
        ax[i][1].set_title(f'{features[i]}')
        ax[i][1].set_ylim(ax0_xlim)

        ax[i][0].set_xlim([0, 127])
        ax[i][1].set_xlim([0, 127])

    # Show the plot
    plt.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.25))
    plt.tight_layout()
    plt.show()
    # plt.savefig('plots/augmented.pdf', dpi=300)


def plot_fft(feat_array: np.ndarray, feat_fft_array: np.ndarray):
    y = feat_array[2000][3]
    y_fft = feat_fft_array[2000][3]

    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15, 5))
    ax[0].plot(y)  # plot time series
    ax[1].stem(y_fft)  # plot freq domain
    plt.tight_layout()
    plt.savefig('plots/fft.pdf', dpi=300)


def smooth(y, box_pts):
    box = np.ones(box_pts) / box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth


def plot_loss_acc(loss_history, acc_history, lr_history):
    f, ax = plt.subplots(3, 1, figsize=(12, 10))

    ax[0].plot(loss_history, label='loss')
    ax[0].set_title('Validation Loss History')
    ax[0].set_xlabel('Epoch no.')
    ax[0].set_ylabel('Loss')

    ax[1].plot(smooth(acc_history, 5)[:-2], label='acc')
    ax[1].set_title('Validation Accuracy History')
    ax[1].set_xlabel('Epoch no.')
    ax[1].set_ylabel('Accuracy')

    ax[2].plot(lr_history, label='lr')
    ax[2].set_title('LR History')
    ax[2].set_xlabel('Epoch no.')
    ax[2].set_ylabel('LR')

    plt.tight_layout()
    plt.show()


def cosine(epoch, t_max, ampl):
    t = epoch % t_max
    return (1 + np.cos(np.pi * t / t_max)) * ampl / 2


def inv_cosine(epoch, t_max, ampl):
    return 1 - cosine(epoch, t_max, ampl)


def one_cycle(epoch, t_max, a1=0.6, a2=1.0, pivot=0.3):
    pct = epoch / t_max
    if pct < pivot:
        return inv_cosine(epoch, pivot * t_max, a1)
    return cosine(epoch - pivot * t_max, (1 - pivot) * t_max, a2)
