function s = detect(agent, landmarks, r_detection)
for ii = 1:size(agent,2)
    d = zeros(1, size(landmarks, 2));
    for a=1:size(landmarks, 2)
        d(a) = vecnorm(agent(:,ii)-landmarks(:,a), 2, 1);
    end
    s(ii,:) = d < r_detection;
end
end