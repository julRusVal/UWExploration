import torch
import gpytorch
from gpytorch.models import ApproximateGP
from gpytorch.variational import CholeskyVariationalDistribution, VariationalStrategy
from gpytorch.likelihoods import GaussianLikelihood
from torch.utils.data import DataLoader, TensorDataset

# Example dataset
def generate_data(n=200):
    x = torch.linspace(0, 1, n).unsqueeze(-1)  # Inputs
    y = torch.sin(2 * 3.1415 * x) + 0.2 * torch.randn_like(x)  # Noisy sine wave
    return x, y

# SVGP Model Definition
class SVGPModel(ApproximateGP):
    def __init__(self, inducing_points, likelihood):
        variational_distribution = CholeskyVariationalDistribution(inducing_points.size(0))
        variational_strategy = VariationalStrategy(
            self,
            inducing_points,
            variational_distribution,
            learn_inducing_locations=True,
        )
        super().__init__(variational_strategy)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())
        self.likelihood = likelihood

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

# Training function
def train_model(model, likelihood, train_loader, optimizer, epochs=50):
    model.train()
    likelihood.train()
    mll = gpytorch.mlls.VariationalELBO(likelihood, model, num_data=len(train_loader.dataset))

    for _ in range(epochs):
        for x_batch, y_batch in train_loader:
            optimizer.zero_grad()
            output = model(x_batch)
            loss = -mll(output, y_batch.squeeze(-1))
            loss.backward()
            optimizer.step()

# Main script
if __name__ == "__main__":
    # Generate synthetic data
    x, y = generate_data()
    dataset = TensorDataset(x, y)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Initial inducing points
    initial_inducing_points = torch.linspace(0, 1, 20).unsqueeze(-1)

    # Instantiate and train the first SVGP
    likelihood = GaussianLikelihood()
    svgp_model = SVGPModel(initial_inducing_points, likelihood)
    optimizer = torch.optim.Adam([
        {'params': svgp_model.parameters()},
        {'params': likelihood.parameters()},
    ], lr=0.01)

    print("Training first SVGP...")
    train_model(svgp_model, likelihood, train_loader)

    # Extract final inducing points from the first SVGP
    final_inducing_points = svgp_model.variational_strategy.inducing_points
    half_inducing_points = final_inducing_points[:final_inducing_points.size(0) // 2]

    # Instantiate the second SVGP with half the inducing points
    svgp_model_2 = SVGPModel(half_inducing_points, likelihood)

    # Train the second SVGP
    print("Training second SVGP with reduced inducing points...")
    train_model(svgp_model_2, likelihood, train_loader)

    # Evaluate the model
    svgp_model_2.eval()
    likelihood.eval()

    test_x = torch.linspace(0, 1, 100).unsqueeze(-1)
    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        predictions = likelihood(svgp_model_2(test_x))

    # Plot results
    import matplotlib.pyplot as plt
    lower, upper = predictions.confidence_region()
    plt.figure(figsize=(8, 6))
    plt.plot(x.numpy(), y.numpy(), 'k*', label="Training Data")
    plt.plot(test_x.numpy(), predictions.mean.numpy(), 'b', label="Prediction")
    plt.fill_between(test_x.squeeze().numpy(), lower.numpy(), upper.numpy(), alpha=0.3, label="Confidence Region")
    plt.legend()
    plt.show()
