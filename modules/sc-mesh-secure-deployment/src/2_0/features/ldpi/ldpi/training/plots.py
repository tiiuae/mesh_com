import os

import matplotlib as mpl
import matplotlib.patheffects as PathEffects
import matplotlib.pyplot as plt
import numpy as np
from cycler import cycler
from scipy.integrate import quad
from scipy.interpolate import interp1d
from scipy.optimize import brentq
from scipy.stats import gaussian_kde
from sklearn.metrics import accuracy_score, f1_score, precision_score, recall_score
from sklearn.metrics import roc_curve, auc


def plot_one_image(data1):
    plt.imshow(data1, interpolation='nearest')
    plt.show()


def plot_np_image(data1, data2):
    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.imshow(data1, interpolation='nearest', cmap='gray')
    ax2.imshow(data2, interpolation='nearest', cmap='gray')
    plt.show()


def roc(scores, labels, plot=True):
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


# Plot normal/abnormal anomaly score distributions
def plot_anomaly_score_dists(model_name, test_scores, labels, name, threshold, add_legend=False):
    # Convert labels to boolean mask
    bool_abnormal = labels.astype(bool)
    bool_normal = ~bool_abnormal
    normal = test_scores[bool_normal]
    abnormal = test_scores[bool_abnormal]

    # Define histogram bins
    bins = np.linspace(np.min(normal), np.percentile(abnormal, 99.95), 150)

    # Setup plot
    plt.style.use('classic')
    px = 1 / plt.rcParams['figure.dpi']  # pixel in inches
    fig, ax = plt.subplots(figsize=(650 * px, 200 * px))

    # Plot histograms
    ax.hist(abnormal, bins=bins, label='Anomaly' if add_legend else '', linewidth=0, color='#DC3912', stacked=True)
    ax.hist(normal, bins=bins, label='Normal Traffic' if add_legend else '', linewidth=0, color='#3366cc', stacked=True,
            alpha=0.9)

    # Set various attributes
    ax.grid(zorder=0)
    ax.set_axisbelow(True)
    ax.set_xlim(np.min(normal), np.percentile(abnormal, 99.95))
    ax.set_ylim(bottom=1)
    ax.set_xlabel('Anomaly Score')
    ax.set_ylabel(r'# of Flows (log)')
    ax.set_yscale('log')

    # Threshold annotation
    plt.axvline(x=threshold, linewidth=1.5, color='black', linestyle='dashed')
    text = plt.text(threshold, ax.get_ylim()[0] + 0.5, 'Threshold', rotation=90, size='small', color='black')
    text.set_path_effects([PathEffects.withStroke(linewidth=1.5, foreground='w')])

    # Add line to max percentile of abnormal
    maxth = np.percentile(abnormal, 99.99)
    plt.text(maxth, ax.get_ylim()[0] + 0.5, 'Max. Abnormal', rotation=90, size='small', color='black')

    if add_legend:
        ax.legend()

    # Your code to create the figure
    fig.tight_layout()

    # Define the plot path and ensure the directory exists
    plot_path = f'output/{model_name}/plots/'
    os.makedirs(plot_path, exist_ok=True)

    # Use os.path.join to create the full file path
    file_path = os.path.join(plot_path, f'{name}.pdf')

    # Save the plot
    plt.savefig(file_path, bbox_inches='tight', format='pdf', dpi=800)

    # Close the figure
    plt.close(fig)


def plot_multiclass_anomaly_scores(test_scores, labels, name, threshold, add_legend=False):
    unique_labels = np.unique(labels)

    # Define colors for each class - adjust this as needed
    colors = plt.cm.get_cmap('tab10', len(unique_labels))

    # Define histogram bins
    bins = np.linspace(np.min(test_scores), np.percentile(test_scores, 99.95), 150)

    # Setup plot
    plt.style.use('classic')
    px = 1 / plt.rcParams['figure.dpi']  # pixel in inches
    fig, ax = plt.subplots(figsize=(650 * px, 200 * px))

    for i, label in enumerate(unique_labels):
        # Extract scores for the current label
        class_scores = test_scores[labels == label]

        # Plot histogram for the current label
        ax.hist(class_scores, bins=bins, label=f'Class {label}' if add_legend else '', linewidth=0, color=colors(i),
                stacked=True, alpha=0.7)

    # Set various attributes
    ax.grid(zorder=0)
    ax.set_axisbelow(True)
    ax.set_xlim(np.min(test_scores), np.percentile(test_scores, 99.95))
    ax.set_ylim(bottom=1)
    ax.set_xlabel('Anomaly Score')
    ax.set_ylabel(r'# of Samples (log)')
    ax.set_yscale('log')

    # Threshold annotation
    plt.axvline(x=threshold, linewidth=1.5, color='#3366cc', linestyle='dashed', label=f'Threshold: {threshold}')
    text = plt.text(threshold, ax.get_ylim()[0] + 0.5, 'Threshold', rotation=90, size='small', color='black')
    text.set_path_effects([PathEffects.withStroke(linewidth=1.5, foreground='w')])

    if add_legend:
        ax.legend()

    fig.tight_layout()
    plt.show()
    plt.close(fig)


def perf_measure(threshold, y_true, scores):
    y_pred = np.empty_like(y_true)
    for i in range(len(y_true)):
        if scores[i] < threshold:
            y_pred[i] = 0
        else:
            y_pred[i] = 1

    # from sklearn.metrics import confusion_matrix
    # tn, fp, fn, tp = confusion_matrix(y_true, y_pred).ravel()
    # print(tn, fp, fn, tp)
    # print('FAR', fp / (fp + tn) * 100)

    accuracy = accuracy_score(y_true, y_pred)
    precision = precision_score(y_true, y_pred, zero_division=0)
    recall = recall_score(y_true, y_pred, zero_division=0)
    f_score = f1_score(y_true, y_pred, zero_division=0)
    return accuracy, precision, recall, f_score


def calculate_overlap_coefficient(normal_scores, abnormal_scores):
    # Estimate the PDFs using Kernel Density Estimation
    kde_normal = gaussian_kde(normal_scores)
    kde_abnormal = gaussian_kde(abnormal_scores)

    # Define a function to compute the minimum of the two PDFs
    min_pdf = lambda x: min(kde_normal(x), kde_abnormal(x))

    # Define the range for integration (cover the range of both score sets)
    min_score = min(normal_scores.min(), abnormal_scores.min())
    max_score = max(normal_scores.max(), abnormal_scores.max())

    # Compute the overlap coefficient by integrating the minimum PDF
    overlap_area, _ = quad(min_pdf, min_score, max_score)
    return overlap_area
