

q1 = [calib.GetPs2dbcg; q(6:end)];
[cost1, jacobian1] = this.CostJointOpt(q1, mk, odo, calib);
[cost2, jacobian2] = this.CostJointOpt2(q, mk, odo, calib);
figure;
plot(cost1-cost2);