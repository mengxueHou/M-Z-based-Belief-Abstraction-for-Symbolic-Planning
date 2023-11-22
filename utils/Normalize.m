function state = Normalize(state)

for tt = 1:size(state,2)

   state(find(state(:,tt)<0), tt) = 0;

    if sum(state(:,tt) ) == 0
        state(:,tt) = ones(size(state(:,tt)))/size(state(:,tt),1);
    end
    state(:,tt) = state(:,tt) / norm(state(:,tt),1);
end

end

