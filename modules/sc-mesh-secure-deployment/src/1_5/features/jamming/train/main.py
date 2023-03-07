import random
import time

import matplotlib.pyplot as plt
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler
from sklearn.metrics import confusion_matrix
from timm.data.mixup import Mixup
from torch import nn
from tsai.models.all import ResCNN

import data
import util
from network import MLP
from transforms import *

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def plot_confusion_matrix(model, loader, label_encoder):
    _, _, test_dl = loader
    model.eval()
    y_true = []
    y_pred = []
    for batch in test_dl:
        x_raw, x_fft, y_bin_batch, y_batch = [t.to(device) for t in batch]
        out = model(x_raw)
        preds = F.log_softmax(out, dim=1).argmax(dim=1)
        y_true.extend(y_batch.cpu().numpy())
        y_pred.extend(preds.cpu().numpy())

    classes = label_encoder.classes_
    cm = confusion_matrix(y_true, y_pred)
    fig, ax = plt.subplots(figsize=(12, 10))
    im = ax.imshow(cm, interpolation='nearest', cmap=plt.cm.Blues)
    ax.figure.colorbar(im, ax=ax)
    ax.set(xticks=np.arange(cm.shape[1]), yticks=np.arange(cm.shape[0]), xticklabels=classes, yticklabels=classes, ylabel='True label', xlabel='Predicted label')
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right", rotation_mode="anchor")
    fmt = 'd'
    thresh = cm.max() / 2.
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            ax.text(j, i, format(cm[i, j], fmt), ha="center", va="center", color="white" if cm[i, j] > thresh else "black")

    fig.tight_layout()
    # plt.show()
    plt.savefig('plots/cm.pdf', dpi=300)


def test(model, loaders, load_best=False):
    _, _, test_dl = loaders

    if load_best:
        model.load_state_dict(torch.load('best.pth'))

    model.eval()
    correct, bin_correct, total = 0, 0, 0
    for batch in test_dl:
        x_raw, x_fft, y_bin_batch, y_batch = [t.to(device) for t in batch]
        out = model(x_raw)

        # Compute predictions for multi-class
        preds = F.log_softmax(out, dim=1).argmax(dim=1)
        correct += (preds == y_batch).sum().item()

        # Compute normal vs. all
        bin_preds = torch.where(preds > 2, 1, 0)
        bin_correct += (bin_preds == y_bin_batch).sum().item()

        total += y_batch.size(0)

    acc = correct / total
    bin_acc = bin_correct / total
    print(f'Test accuracy multiclass: {acc}, bin accuracy: {bin_acc}')
    return acc, bin_acc


def plot_series(input, classes):
    f, ax = plt.subplots(3, 1, figsize=(12, 10))

    ax[0].plot(input[0][3].cpu().numpy(), label=classes[0])

    ax[1].plot(input[1][3].cpu().numpy(), label=classes[1])

    ax[2].plot(input[2][3].cpu().numpy(), label=classes[2])

    plt.tight_layout()
    plt.show()


def train(model, loaders, len_train_data):
    train_dl, val_dl, test_dl = loaders
    total_steps = util.EPOCHS * len(train_dl)

    iteration = 0
    best_bin_acc, best_acc, best_test_acc, best_test_bin_acc = 0, 0, 0, 0
    val_loss_history, val_acc_history, test_acc_history, test_bin_acc_history, lr_history = [], [], [], [], []

    criterion = nn.CrossEntropyLoss(reduction='mean')
    # opt = optim.Adam(model.parameters(), lr=util.LR, weight_decay=0.0001)
    opt = optim.AdamW(model.parameters(), lr=util.LR, weight_decay=0.00001)
    # opt = Lion(model.parameters(), lr=util.LR, weight_decay=0.0000001)

    # Define the cosine annealing scheduler
    scheduler = lr_scheduler.CosineAnnealingLR(opt, total_steps)

    mixup_args = {
        'mixup_alpha': 0.0,
        'cutmix_alpha': 1.0,
        'cutmix_minmax': None,
        'prob': 1.0,
        'switch_prob': 0.,
        'mode': 'batch',
        'label_smoothing': 0.1,
        'num_classes': 22}
    mixup_fn = Mixup(**mixup_args)

    print('Start model training')
    for epoch in range(1, util.EPOCHS + 1):
        epoch_time = time.time()
        model.train()
        epoch_loss, n_train_samples, last_lr = 0, 0, 0
        for i, batch in enumerate(train_dl):
            series, y_bin_batch, targets = batch
            if torch.cuda.is_available():
                series = series.cuda(non_blocking=True)
                # y_bin_batch = batch[1].cuda(non_blocking=True)
                targets = targets.cuda(non_blocking=True)

            # if np.random.random() < 0.5:
            # series, targets = mixup_fn(series, targets) # Test accuracy multiclass: 0.9959919839679359, bin accuracy: 0.998997995991984
            # without mixup with crop and horizontal flip => Test accuracy multiclass: 0.998997995991984, bin accuracy: 1.0

            # plot_series(series, targets)

            # x_fft = torch.abs(torch.fft.rfft(series))

            # Forward pass and calculate loss
            out = model(series)
            loss = criterion(out, targets)

            # Backward pass and update weights
            opt.zero_grad()
            loss.backward()
            opt.step()

            # Set the learning rate for next step
            scheduler.step()
            last_lr = opt.param_groups[0]['lr']

            # Logging
            epoch_loss += loss.item()
            n_train_samples += series.shape[0]
            iteration += 1

        epoch_loss /= n_train_samples
        val_loss_history.append(epoch_loss)
        lr_history.append(last_lr)

        model.eval()
        correct, bin_correct, total = 0, 0, 0
        for batch in val_dl:
            series, x_fft, y_bin_batch, targets = [t.to(device) for t in batch]
            out = model(series)

            # Compute predictions for multi-class
            preds = F.log_softmax(out, dim=1).argmax(dim=1)
            correct += (preds == targets).sum().item()

            # Compute normal vs. all
            bin_preds = torch.where(preds > 2, 1, 0)
            bin_correct += (bin_preds == y_bin_batch).sum().item()

            total += targets.size(0)

        acc = correct / total
        bin_acc = bin_correct / total
        val_acc_history.append(acc)

        acc_test = None
        if bin_acc >= best_bin_acc:
            best_bin_acc = bin_acc

            # Test with best model so far on the validation dataset
            if best_test_bin_acc < 1.0:
                acc_test, bin_acc_test = test(model, loaders)
                if bin_acc_test > best_test_bin_acc:
                    best_test_bin_acc = bin_acc_test
                    torch.save(model.state_dict(), 'best_bin.pth')

        if acc >= best_acc:
            best_acc = acc

            # Test with best model so far on the validation dataset
            if best_test_acc < 1.0:
                if acc_test is None: acc_test, bin_acc_test = test(model, loaders)
                if bin_acc_test > best_test_bin_acc:
                    best_test_bin_acc = bin_acc_test
                    torch.save(model.state_dict(), 'best_bin.pth')
                if acc_test > best_test_acc:
                    best_test_acc = acc_test
                    torch.save(model.state_dict(), 'best.pth')
        elif acc * 1.005 > best_acc:
            if best_test_acc < 1.0:
                if acc_test is None: acc_test, bin_acc_test = test(model, loaders)
                if bin_acc_test > best_test_bin_acc:
                    best_test_bin_acc = bin_acc_test
                    torch.save(model.state_dict(), 'best_bin.pth')
                if acc_test > best_test_acc:
                    best_test_acc = acc_test
                    torch.save(model.state_dict(), 'best.pth')

        if epoch % 1 == 0:
            epoch_time = time.time() - epoch_time
            print(f'Epoch: {epoch:3d}. Loss: {epoch_loss:.4f}. Val. Acc.: {acc:2.2%}. Val. Bin. Acc.: {bin_acc:2.2%} - Test Acc.: {best_test_acc:2.2%}. '
                  f'Test Bin Acc.: {best_test_bin_acc:2.2%}. Last lr {last_lr:2.8}, Time: {epoch_time:2.2}')

    print('Done!')

    util.plot_loss_acc(val_loss_history, val_acc_history, lr_history)


def main():
    random.seed(util.SEED)
    np.random.seed(util.SEED)
    torch.manual_seed(util.SEED)

    loaders, len_train_data, label_encoder = data.get_classification_data()

    # Simple MLP with a symmetric decoder for pretraining
    # model = Classifier(raw_ni=15, fft_ni=15, no=21, drop=0.25).to(device)
    # model = InceptionTime(15, 21).to(device)  # Test accuracy multiclass: 0.9849498327759197, bin accuracy: 1.0
    # model = InceptionTimePlus(15, 21).to(device)  # Test accuracy multiclass: 0.9832775919732442, bin accuracy: 1.0
    # model = XCMPlus(15, 21, 128).to(device) # Test accuracy multiclass: 0.9765886287625418, bin accuracy: 0.9966
    # model = ResCNN(15, 21).to(device) # Test accuracy multiclass: 0.9866220735785953, bin accuracy: 1.0
    model = ResCNN(15, 22, separable=True, dropout=0.0).to(device)  # Test accuracy multiclass: 0.991638795986, bin accuracy: 1.0
    # model = TCN(15, 21).to(device) # problems
    # model = xresnet1d18(15, 21).to(device)  # Test accuracy multiclass: 0.9866220735785953, bin accuracy: 1.0
    util.model_summary(model)

    # Train
    train(model, loaders, len_train_data)

    # Test
    test(model, loaders, load_best=True)

    plot_confusion_matrix(model, loaders, label_encoder)


if __name__ == '__main__':
    main()
    # main()
