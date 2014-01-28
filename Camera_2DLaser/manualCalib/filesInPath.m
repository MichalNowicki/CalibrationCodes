% function cellFiles = filesInPath(pathString)
% returns a cell array containing all the files in pathString which end
% in '.gz'
% function cellFiles = filesInPath(pathString, extString)
% returns a cell array containing all the files in pathString which end
% in extString

function cellFiles = filesInPath(pathString, varargin)
  if length(varargin) > 0
    extString = lower(varargin{1});
  else
    extString = '.gz';
  end
  dirList = dir(pathString);
  cellFiles = {};
  jx = 1;
  for ix=1:length(dirList)
    fn = lower(dirList(ix).name);
      idxes= findstr(fn, extString);
      if not(isempty(idxes))
	if idxes(end) == (length(fn) - length(extString) + 1)
	  cellFiles{jx} = [pathString '/' fn];
	  jx = jx+1;
	end
    end
  end
  
  filenameSize = size(cellFiles{1});
  allOK = 1;
  for i = 2:length(cellFiles);
    if any(filenameSize ~= size(cellFiles{i}))
      allOK = 0;
    end
  end
  if (allOK)
    %disp('sorting filenames lexicographically');
    for i = 1:length(cellFiles);
      filenames(i,:) = cellFiles{i};
    end
    [jnk,idxes] = sortrows(filenames);
    for i = 1:length(cellFiles);
      cellFiles{i} = filenames(idxes(i),:);
    end
  else
    disp(['filenames of different lengths.  Unable to sort' ...
          ' lexicographically, Matlab inherent order is by creation date']);
  end
 
