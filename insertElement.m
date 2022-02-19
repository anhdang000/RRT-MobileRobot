function arr = insertElement(arr, element, idx)
n_elements = size(arr, 1);

if idx == 1
    % head
    arr = [element; arr];
elseif idx == n_elements + 1
    % tail
    arr = [arr; element];
else
    % mid
    arr = [arr(1:idx-1, :); element; arr(idx:end, :)];
end

end

