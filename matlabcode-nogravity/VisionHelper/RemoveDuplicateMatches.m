function [indexPairs, score] = RemoveDuplicateMatches(indexPairs, score)
    A= indexPairs(:,2);
    
    uniqueA = unique(A);
    [n, bin] = histc(A, uniqueA);
    multiple = find(n>1);
    n_repetition = length(multiple);    

    if n_repetition ~= 0
        index = find(ismember(bin, multiple));
        repetitions = A(index);
        
        temp = sortrows([repetitions, index]);
        pointer = 1;
        rowsToDelete=[];
        for i=1:n_repetition
            repeated_number = temp(pointer,1);
            n_appearance = sum(repeated_number == temp(:,1));
            repeated_idx = temp(pointer:pointer+n_appearance-1, 2);
            
            [~, best_idx] = max(score(repeated_idx));
            repeated_idx(best_idx) = [];
            rowsToDelete = [rowsToDelete, repeated_idx'];
            
            
            pointer = pointer + n_appearance;
        end
        indexPairs(rowsToDelete,:) = [];
        score(rowsToDelete)=[];
    end
        
end