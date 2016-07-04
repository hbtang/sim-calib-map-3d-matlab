rng('default');  % For reproducibility
X = randn(100,25);

figure;

subplot(2,1,1)
boxplot(X)

subplot(2,1,2)
boxplot(X,'plotstyle','compact')