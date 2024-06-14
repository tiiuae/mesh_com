import os
import time
from typing import Optional, Tuple, Dict, NoReturn

import numpy as np
import torch
import torch.optim as optim
import torch.quantization
from torch import Tensor
from torch.nn import Module
from torch.optim import SGD
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader
from tqdm import tqdm

import data
import plots
from losses import OneClassContrastiveLoss
from model import ResCNNContrastive
from options import LDPIOptions


class ContrastivePretrainer:
    """
    Class for contrastive pretraining.
    """

    def __init__(self, args: LDPIOptions, model: ResCNNContrastive, data_loader: DataLoader, initial_learning_rate: float = 0.1, warmup_epochs: int = 100) -> NoReturn:
        """
        Initialize the ContrastivePretrainer.

        Args:
            args (LDPIOptions): The LDPIOptions object containing pretraining options.
            model (ResCNNContrastive): The contrastive model to be pretrained.
            data_loader (DataLoader): DataLoader for loading data.
            initial_learning_rate (float, optional): Initial learning rate. Defaults to 0.1.
            warmup_epochs (int, optional): Number of warmup epochs. Defaults to 100.
        """
        self.args = args
        self.model = model
        self.data_loader = data_loader
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.initial_learning_rate = initial_learning_rate
        self.warmup_epochs = warmup_epochs

    def pretrain(self):
        """
        Perform the contrastive pretraining.
        """
        self._setup_training()
        self._execute_training()

    def _setup_training(self):
        """
        Setup training components.
        """
        self.optimizer = SGD(self.model.parameters(), lr=self.initial_learning_rate, weight_decay=0.0003)
        self.loss_function = OneClassContrastiveLoss(tau=0.2)
        self.scheduler = CosineAnnealingLR(self.optimizer, T_max=self.args.pretrain_epochs - self.warmup_epochs, eta_min=0.0)

    def _execute_training(self):
        """
        Execute the training loop.
        """
        with tqdm(total=self.args.pretrain_epochs, desc="Training Progress") as progress_bar:
            for epoch in range(self.args.pretrain_epochs):
                self._apply_warmup(epoch)

                batch_count, total_loss = self._train_single_epoch()

                if epoch >= self.warmup_epochs:
                    self.scheduler.step()

                self._update_progress_bar(progress_bar, epoch, batch_count, total_loss)

    def _apply_warmup(self, current_epoch: int) -> NoReturn:
        """
        Apply learning rate warmup.

        Args:
            current_epoch (int): Current epoch.
        """
        if current_epoch < self.warmup_epochs:
            warmup_learning_rate = ((self.initial_learning_rate - 1e-10) / self.warmup_epochs) * current_epoch + 1e-10
            for param_group in self.optimizer.param_groups:
                param_group['lr'] = warmup_learning_rate

    def _train_single_epoch(self) -> Tuple[int, float]:
        """
        Train for a single epoch.

        Returns:
            Tuple[int, float]: Batch count and total loss.
        """
        batch_count, total_loss = 0, 0
        for view_1, view_2 in self.data_loader:
            view_1, view_2 = self._prepare_data_for_device(view_1, view_2)

            loss = self._compute_loss(view_1, view_2)

            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

            total_loss += loss.item()
            batch_count += 1
        return batch_count, total_loss

    def _compute_loss(self, view_1: torch.Tensor, view_2: torch.Tensor) -> torch.Tensor:
        """
        Compute the contrastive loss.

        Args:
            view_1 (torch.Tensor): Input view 1.
            view_2 (torch.Tensor): Input view 2.

        Returns:
            torch.Tensor: Contrastive loss.
        """
        concatenated_views = torch.cat((view_1, view_2), dim=0)
        features = self.model(concatenated_views)
        split_features = torch.stack(features.split(features.size(0) // 2), dim=1)
        return self.loss_function(split_features)

    def _prepare_data_for_device(self, view_1: torch.Tensor, view_2: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Prepare data for the device.

        Args:
            view_1 (torch.Tensor): Input view 1.
            view_2 (torch.Tensor): Input view 2.

        Returns:
            Tuple[torch.Tensor, torch.Tensor]: Prepared data for the device.
        """
        view_1, view_2 = view_1.to(self.device), view_2.to(self.device)
        return view_1.unsqueeze(1), view_2.unsqueeze(1)

    def _update_progress_bar(self, progress_bar: tqdm, epoch: int, batch_count: int, total_loss: float) -> NoReturn:
        """
        Update the training progress bar.

        Args:
            progress_bar (tqdm): Progress bar object.
            epoch (int): Current epoch.
            batch_count (int): Batch count.
            total_loss (float): Total loss.
        """
        learning_rate = self.optimizer.param_groups[0]["lr"]
        mean_loss = total_loss / batch_count
        progress_bar.set_description(f"Epoch: {epoch + 1}, LR: {learning_rate:.5f}, Mean loss: {mean_loss:.5f}")
        progress_bar.update()


class Trainer:
    """
    A class for training and testing a model.
    """

    def __init__(self, ) -> NoReturn:
        """
        Initializes the Trainer class.
        """
        self.args = LDPIOptions()
        self.train_loader: Optional[DataLoader] = None
        self.test_loader: Optional[DataLoader] = None
        self.device: torch.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = ResCNNContrastive().to(self.device)
        self.center: Optional[Tensor] = None
        self.max_best_dr = None
        self.best_sep_score = None
        self.best_model_state = None

    def pretrain(self) -> NoReturn:
        """
        Pretrains the model.
        """
        # Generate data, create datasets and dataloaders
        loader = data.get_pretrain_dataloader(dataset='TII-SSRC-23', batch_size=self.args.batch_size, contrastive=True)

        # Check if the pretrained model exists, else pretrain it
        model_path = f'output/{self.args.model_name}/pretrained_model.pth'
        if os.path.exists(model_path):
            print("Pretrained model found. Loading model...")
            try:
                self.model.load_state_dict(torch.load(model_path))
                print("Model loaded successfully.")
            except RuntimeError as e:
                print("Error loading the model:", e)
        else:
            # Pretrain using one class contrastive approach
            pretrainer = ContrastivePretrainer(self.args, self.model, loader)
            pretrainer.pretrain()

            # Create directory if not exist and then save pretrained model
            os.makedirs(os.path.dirname(model_path), exist_ok=True)
            torch.save(self.model.state_dict(), model_path)

        loader = data.get_pretrain_dataloader(dataset='TII-SSRC-23', batch_size=self.args.batch_size, contrastive=False, shuffle=False, drop_last=False)
        self._init_center_c(loader)

    def train(self, eta: float = 1.0, eps: float = 1e-10, per_validation: int = 5) -> NoReturn:
        """
        Trains the model.

        Args:
            eta (float): A coefficient for the loss function.
            eps (float): A small epsilon value.
            per_validation (int): The number of epochs between validation steps.
        """
        if self._load_model_and_data():
            return

        self._configure_optimizer_scheduler()

        for epoch in range(self.args.epochs):
            self._train_epoch(eta, eps, epoch)
            self._validate_and_save_model(epoch, per_validation)

        # Load the best model if found
        if self.best_model_state:
            print('Loading best model')
            self.model.load_state_dict(self.best_model_state)

    def test(self, plot: bool = False) -> Dict:
        """
        Tests the model.

        Args:
            plot (bool): Whether to plot the results.

        Returns:
            Dict: A dictionary containing test results.
        """
        self.model.eval()
        dataset_size = len(self.test_loader.dataset)
        scores = torch.zeros(size=(dataset_size,), dtype=torch.float32, device=self.device)
        bin_labels = torch.zeros(size=(dataset_size,), dtype=torch.long, device=self.device)
        mult_labels = torch.zeros(size=(dataset_size,), dtype=torch.long, device=self.device)

        with torch.no_grad():
            start, num_samples = time.time(), 0
            for i, (inputs, targets, bin_targets) in enumerate(self.test_loader):
                inputs, targets, bin_targets = inputs.to(self.device).unsqueeze(1), targets.to(self.device), bin_targets.to(self.device)

                outputs = self.model.encode(inputs)

                dist = torch.sum((outputs - self.center) ** 2, dim=1)
                c_targets = torch.where(bin_targets == 1, 0, 1)

                batch_index = i * self.args.batch_size
                scores[batch_index: batch_index + self.args.batch_size] = dist
                bin_labels[batch_index: batch_index + self.args.batch_size] = c_targets
                mult_labels[batch_index: batch_index + self.args.batch_size] = targets
                num_samples += targets.size(0)
            end = time.time()
            print(f'Number of samples: {num_samples}, Inference time: {end - start}, Freq {num_samples / (end - start)}')
        scores_np = scores.to('cpu').numpy()
        bin_labels_np = bin_labels.to('cpu').numpy()
        mult_labels_np = mult_labels.to('cpu').numpy()

        auroc = plots.roc(scores_np, bin_labels_np, plot=False)
        results = {'auc': auroc}

        bool_abnormal = bin_labels_np.astype(bool)
        bool_normal = ~bool_abnormal
        normal = scores_np[bool_normal]
        abnormal = scores_np[bool_abnormal]
        results['separation_score'] = abnormal.min() - normal.max()

        # Compute 99.99th threshold and max threshold
        ninety_nine_threshold = np.percentile(normal, 99)
        ninety_nine_threshold = np.nextafter(ninety_nine_threshold, np.inf)
        near_max_threshold = np.percentile(normal, 99.99)
        near_max_threshold = np.nextafter(near_max_threshold, np.inf)
        max_threshold = np.max(normal)
        max_threshold = np.nextafter(max_threshold, np.inf)
        hundred_one_threshold = max_threshold * 1.01

        for name, threshold in [('ninety_nine', ninety_nine_threshold), ('near_max', near_max_threshold), ('max', max_threshold), ('hundred_one', hundred_one_threshold)]:
            acc, prec, rec, f_score = plots.perf_measure(threshold, bin_labels_np, scores_np)
            results[name] = {'threshold': threshold, 'acc': acc, 'prec': prec, 'rec': rec, 'f_score': f_score}

            if plot:
                # Calculate 4 * max_threshold for plotting
                right_limit = np.percentile(abnormal, 99.99)

                # Filter out scores and corresponding labels that are above 4 * max_threshold for plotting
                valid_indices = scores_np <= right_limit
                plot_scores_np = scores_np[valid_indices]
                plot_bin_labels_np = bin_labels_np[valid_indices]

                plots.plot_anomaly_score_dists(self.args.model_name, test_scores=plot_scores_np, labels=plot_bin_labels_np, name=name, threshold=threshold)

        return results

    def trace_and_measure_inference(self):
        """
        Traces the model and measures inference time before and after tracing.

        Returns:
            Tuple[float, float]: Inference frequencies before and after tracing.
        """
        self._move_all_cpu()
        self.model.eval()

        # Measure inference time before tracing
        time_pre_trace, total_samples_pre = self._measure_inference_time(self.model)
        freq_pre_trace = total_samples_pre / time_pre_trace

        # Trace the encode method
        sample_inputs, _, _ = next(iter(self.test_loader))
        sample_inputs = sample_inputs.unsqueeze(1).to(self.device)
        first_input = sample_inputs[0:1]
        traced_model = torch.jit.trace_module(
            self.model,
            {'encode': first_input}
        )

        # Create directory and save model
        traced_model_path = f'output/{self.args.model_name}'
        os.makedirs(traced_model_path, exist_ok=True)
        model_save_path = os.path.join(traced_model_path, 'traced_model.pth')
        torch.jit.save(traced_model, model_save_path)

        # Load the traced model
        traced_model = torch.jit.load(f'{traced_model_path}/traced_model.pth')

        # Measure inference time after tracing
        time_post_trace, total_samples_post = self._measure_inference_time(traced_model)
        freq_post_trace = total_samples_post / time_post_trace

        return freq_pre_trace, freq_post_trace

    def quantize_model(self, backend: str = 'qnnpack'):  # 'qnnpack' or 'x86
        """
        Quantizes the model.

        Args:
            backend (str): The backend for quantization ('qnnpack' or 'x86').
        """
        self._move_all_cpu()
        self.model.eval()

        # Set the backend for quantization
        torch.backends.quantized.engine = backend

        # Configure the model for quantization
        self.model.qconfig = torch.quantization.get_default_qconfig(backend)

        # Prepare the model for quantization
        torch.quantization.prepare(self.model, inplace=True)

        # Calibrate the model using the test loader
        with torch.no_grad():
            for i, (inputs, targets, bin_targets) in enumerate(self.test_loader):
                inputs = inputs.to(self.device).unsqueeze(1)
                self.model.encode(inputs)

        # Convert the model to a quantized state
        quantized_model = torch.quantization.convert(self.model, inplace=True)
        self.model = quantized_model

        # Trace the quantized model using jit.script
        first_input = inputs[0:1]
        print(first_input.shape)
        scripted_quantized_model = torch.jit.script(quantized_model)

        # Save the scripted quantized model to the specified output folder
        output_folder = f'output/{self.args.model_name}'
        os.makedirs(output_folder, exist_ok=True)
        output_path = os.path.join(output_folder, 'scripted_quantized_model.pth')
        torch.jit.save(scripted_quantized_model, output_path)

    def _init_center_c(self, loader: DataLoader, apply_threshold: bool = False, eps: float = 0.01) -> NoReturn:
        """
        Initializes the center vector.

        Args:
            loader (DataLoader): DataLoader for pretraining data.
            apply_threshold (bool): Whether to apply an epsilon threshold to the center vector.
            eps (float): A small epsilon value.
        """
        self.center = torch.zeros(self.model.feat_dim, device=self.device)

        self.model.eval()
        with torch.no_grad():
            total_outputs, n_samples = 0, 0
            for inputs, *_ in loader:
                inputs = inputs.unsqueeze(1).to(self.device)

                # Encode the inputs and unsqueeze to get [bs, 1, feat] shape
                outputs = self.model.encode(inputs.to(self.device))

                # Sum the outputs for the current batch and add to the total
                total_outputs += outputs.sum(0)
                n_samples += outputs.size(0)

        self.center = total_outputs / n_samples

        # Apply epsilon threshold
        if apply_threshold:
            # Create a mask for values less than epsilon
            eps_mask = np.abs(self.center) < eps

            # Apply -eps to elements where mask is True and value is negative
            self.center = np.where(eps_mask & (self.center < 0), -eps, self.center)

            # Apply eps to elements where mask is True and value is positive
            self.center = np.where(eps_mask & (self.center > 0), eps, self.center)

        print(f'Computed center: {self.center}')

    def _load_model_and_data(self):
        """
        Loads the model and data.

        Returns:
            bool: True if a saved model state is found, False otherwise.
        """
        self.train_loader, self.test_loader = data.get_training_dataloader(dataset='TII-SSRC-23')
        model_save_path = f'output/{self.args.model_name}/best_model_with_center.pth'
        if os.path.isfile(model_save_path):
            print('Loading best model and center from saved state')
            saved_state = torch.load(model_save_path)
            self.model.load_state_dict(saved_state['model_state_dict'])
            self.center = saved_state['center']
            print(self.center)
            return True
        return False

    def _configure_optimizer_scheduler(self):
        """
        Configures the optimizer and scheduler.
        """
        initial_lr = 0.01
        self.optimizer = optim.SGD(self.model.parameters(), lr=initial_lr, weight_decay=0.0003)
        self.warmup_epochs = min(100, int(0.1 * self.args.epochs))
        self.warmup_lr = 1e-10
        self.lr_increment = (initial_lr - self.warmup_lr) / self.warmup_epochs
        self.scheduler = CosineAnnealingLR(self.optimizer, T_max=self.args.epochs - self.warmup_epochs, eta_min=0.0)
        self.continue_warmup = True

    def _train_epoch(self, eta, eps, epoch):
        """
        Trains the model for one epoch.

        Args:
            eta (float): A coefficient for the loss function.
            eps (float): A small epsilon value.
            epoch (int): The current epoch.
        """
        self.model.train()
        epoch_loss = 0.0
        n_batches = 0
        epoch_start_time = time.time()

        # Warmup LR
        if self.continue_warmup and epoch < self.warmup_epochs:
            self.warmup_lr += self.lr_increment
            for param_group in self.optimizer.param_groups:
                param_group['lr'] = self.warmup_lr

        # Training for each batch
        for inputs, targets, bin_targets in self.train_loader:
            self.optimizer.zero_grad()

            inputs, bin_targets = inputs.to(self.device).unsqueeze(1), bin_targets.to(self.device)

            outputs = self.model.encode(inputs)
            dist = torch.sum((outputs - self.center) ** 2, dim=1)
            losses = torch.where(bin_targets == 0, dist, eta * ((dist + eps) ** bin_targets.float()))
            loss = torch.mean(losses)

            loss.backward()
            self.optimizer.step()

            epoch_loss += loss.item()
            n_batches += 1

        # Update learning rate with cosine annealing after warmup
        if epoch >= self.warmup_epochs:
            self.scheduler.step()
        current_lr = self.optimizer.param_groups[0]['lr']

        # Logging epoch statistics
        epoch_train_time = time.time() - epoch_start_time
        print(f'| Epoch: {epoch + 1:03}/{self.args.epochs:03} | Train Time: {epoch_train_time:.3f}s | Train Loss: {epoch_loss / n_batches:.6f} | LR: {current_lr}')

    def _validate_and_save_model(self, epoch: int, per_validation: int) -> NoReturn:
        """
        Periodically validates and saves the best model based on detection rates and separation score.

        Args:
            epoch (int): The current epoch.
            per_validation (int): The number of epochs between each validation.
        """
        # Periodic testing
        if (epoch + 1) % per_validation == 0 or epoch < self.warmup_epochs:
            print("Validation for early stopping at epoch:", epoch + 1)
            results = self.test(plot=False)

            max_dr = results['max']['rec']
            separation_sccore = results['separation_score']
            print('max', max_dr)
            print('separation_sccore', separation_sccore)

            # Compare current model's performance with the best so far
            if self.max_best_dr is None or max_dr > self.max_best_dr or (max_dr == self.max_best_dr and separation_sccore > self.best_sep_score):
                # Update the best detection rates
                self.max_best_dr = max_dr
                self.best_sep_score = separation_sccore

                # Save the current model as the best model
                self.best_model_state = self.model.state_dict().copy()
                print("New best model found!")

                # Save best model
                output_dir = f'output/{self.args.model_name}'
                os.makedirs(output_dir, exist_ok=True)
                save_dict = {
                    'model_state_dict': self.best_model_state,
                    'center': self.center,
                    'results': results
                }

                model_save_path = os.path.join(output_dir, 'best_model_with_center.pth')
                torch.save(save_dict, model_save_path)
                self.continue_warmup = True
            else:
                self.continue_warmup = False

    def _measure_inference_time(self, model: Module) -> Tuple[float, int]:
        """
        Measures the inference time of a given model on a test dataset.

        Args:
            model (Module): The model to measure inference time for.

        Returns:
            Tuple[float, int]: A tuple containing the inference duration in seconds and the total number of samples processed.
        """
        start_time = time.time()
        total_samples = 0
        with torch.no_grad():
            for i, (inputs, _, _) in enumerate(self.test_loader):
                inputs = inputs.unsqueeze(1).to(self.device)
                model.encode(inputs)
                total_samples += inputs.size(0)
        end_time = time.time()
        duration = end_time - start_time
        print(f"Processed {total_samples} samples in {duration} seconds.")
        return duration, total_samples

    def _move_all_cpu(self):
        self.device = torch.device('cpu')
        self.model = self.model.to(self.device)
        self.center = self.center.to(self.device)


def main() -> NoReturn:
    """
    Main function to perform training, testing, tracing, and quantization of the model.
    """
    # Create a Trainer instance
    trainer = Trainer()

    # Pretrain the model
    trainer.pretrain()

    # Train the model further
    trainer.train()

    # Testing the trained model
    results = trainer.test(plot=True)
    print("Test results:", results)

    # Tracing and measuring inference before and after model tracing
    freq_pre_trace, freq_post_trace = trainer.trace_and_measure_inference()
    print(f'Inference Frequency - Before Model Tracing: {freq_pre_trace} - After Model Tracing: {freq_post_trace}')

    # Quantize the trained model
    trainer.quantize_model()


if __name__ == '__main__':
    main()
