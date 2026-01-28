
function avg_diff_cov_jac(resdir)
% Compute and visualize average diff matrices for:
%   covGtsam_* vs covRK4_*
%   jacobianGtsam_* vs jacobianRK4_*
if nargin < 1 || isempty(resdir)
    resdir = '/media/jhuai/T5EVO/jhuai/results/isprs2026-preint';
end

tol      = 1e-3;
startIdx = 0;
endIdx   = 99;
numInst  = endIdx - startIdx + 1;

% Helper that knows about resdir
computeAvgDiff = @(refBase, estBase, refmest) ...
    local_compute_avg_diff(resdir, refBase, estBase, startIdx, endIdx, tol, refmest);

% Average diff for covariance matrices
avgCovDiff = computeAvgDiff('covGtsam', 'covRK4', true);

% Average diff for Jacobian matrices
avgJacDiff = computeAvgDiff('jacobianGtsam', 'jacobianRK4', false);

%% Figure 1: Covariance diff
fig1 = figure;
imagesc(avgCovDiff);
axis image;
colorbar;
colormap('jet');
title(sprintf('Avg cov diff (%d instances)', numInst));
xlabel('Column');
ylabel('Row');

cov_png = fullfile(resdir, 'avgCovDiff.png');
exportgraphics(gcf, cov_png, 'Resolution', 300);  % cropped, high-res PNG


%% Figure 2: Jacobian diff
fig2 = figure;
imagesc(avgJacDiff);
axis image;
colorbar;
colormap('jet');
title(sprintf('Avg Jacobian diff (%d instances)', numInst));
xlabel('Column');
ylabel('Row');

jac_png = fullfile(resdir, 'avgJacDiff.png');
exportgraphics(gcf, jac_png, 'Resolution', 300);  % cropped, high-res PNG

fprintf('Saved:\n  %s\n  %s\n', cov_png, jac_png);

% ========================================================================
% Local function
% ========================================================================
function avgR = local_compute_avg_diff(resdir, refBase, estBase, startIdx, endIdx, tol, refmest)
    numInst = endIdx - startIdx + 1;

    % First pair to init
    ref0 = load(fullfile(resdir, sprintf('%s_%d.txt', refBase, startIdx)));
    est0 = load(fullfile(resdir, sprintf('%s_%d.txt', estBase, startIdx)));

    if ~isequal(size(ref0), size(est0))
        error('First %s / %s matrices have different sizes.', refBase, estBase);
    end

    sumR = zeros(size(ref0));

    for k = startIdx:endIdx
        ref = load(fullfile(resdir, sprintf('%s_%d.txt', refBase, k)));
        est = load(fullfile(resdir, sprintf('%s_%d.txt', estBase, k)));

        if ~isequal(size(ref), size(est))
            error('Size mismatch at index %d for %s / %s', k, refBase, estBase);
        end

        % if |ref| > tol: r = (est - ref) ./ ref
        % else          : r = est - ref
        mask = abs(ref) > tol;
        r    = zeros(size(ref));
        r(mask)  = (ref(mask) - est(mask)) ./ ref(mask);
        r(~mask) =  ref(~mask) - est(~mask);
        
        if ~refmest
            r = -r;
        end
        sumR = sumR + r;
    end

    avgR = sumR / numInst;
end
end