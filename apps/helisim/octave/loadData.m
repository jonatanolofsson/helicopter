function data = loadData(filename)
    rawData = load(filename);
    data.time = rawData(:,1);
    data.states = rawData(:,2:4)';
    data.reference = rawData(:,5:7)';
    data.control = rawData(:,8:10)';
