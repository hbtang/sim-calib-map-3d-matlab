function DrawPlotEnv( rec_mu, rec_sigma, vec_id, rec_lp )
%DRAWPLOTENV plot rec_plgc with envelop of +-3*std

% if draw cov envolop
b_draw_cov = true;

sz = size(rec_mu);

if nargin == 2
    vec_id = (1:sz(2)).';
end

sz = size(rec_mu);
rec_env_top = [];
rec_env_btn = [];

for i = 1:sz(1)
    std_tmp = zeros(numel(vec_id),1);
    sigma_tmp = rec_sigma{i};
    for j = 1:numel(vec_id)
        std_tmp(j) = sqrt(sigma_tmp(vec_id(j),vec_id(j)));
    end
    rec_env_top = [rec_env_top; rec_mu(i,vec_id) + std_tmp.'*3];
    rec_env_btn = [rec_env_btn; rec_mu(i,vec_id) - std_tmp.'*3];
end


if nargin == 3
    figure;
    grid on; hold on;
    f = plot(rec_mu(:,vec_id), 'LineWidth',2);
    h = legend(f);
    set(h,'Fontsize',10);
    
    if b_draw_cov
        plot(rec_env_top, '--');
        plot(rec_env_btn, '--');
    end
else
    figure;
    grid on; hold on;
    f = plot(floor(rec_lp/30) ,rec_mu(:,vec_id), 'LineWidth',2);
    h = legend(f);
    set(h,'Fontsize',10);
    
    if b_draw_cov
        plot(floor(rec_lp/30), rec_env_top, '--');
        plot(floor(rec_lp/30), rec_env_btn, '--');
    end
end

set(gca,'FontSize',10);





end

