function Data=readLaserLog(filename)

fid = fopen(filename);
Data = fscanf(fid,'%f',[1 inf]);
fclose(fid);
