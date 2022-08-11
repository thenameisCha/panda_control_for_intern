close all; clear all; clc;
    fileID = fopen('build/ETank_data.txt', 'r');
    formatspec = '%f %f %f %f %f';
    sizeB = [6, inf];
    B = fscanf(fileID, formatspec, sizeB);
    fclose(fileID);
    B = B';
    figure(1);
    plot(B(:,1), B(:,2));
    hold on;
    plot(B(:,1), B(:,3));
    legend("x desired dot", "x dot");
    xlabel("play time, [s]");
    ylabel("task space Z velocity, [m/s]");
    title("ETank applied");

    figure(2);
    plot(B(:,1), B(:,4));
    hold on;
    plot(B(:,1), B(:,5));
    xlabel("play time, [s]");
    ylabel("Gamma");
    legend("x_(dot) gamma", "x_(desired, dot) gamma");
    title("ETank applied");

    figure(3)
    plot(B(:,1), B(:,6));
    xlabel("play time, [s]");
    ylabel("S_ur [J]");
    title("ETank applied");

    fileID = fopen('build/Power_data.txt', 'r');
    formatspec = '%f %f %f %f %f';
    sizeC = [5, inf];
    C = fscanf(fileID, formatspec, sizeC);
    fclose(fileID);
    C = C';
    figure(4);
    plot(C(:,1), C(:,2));
    hold on;
    plot(C(:,1), C(:,3));
    hold on;
    plot(C(:,1), C(:,4));
    legend("x_(dot)*F_F", "x_(desired, dot)*F_F", "x_(desired, dot)*F_ext");
    xlabel("play time, [s]");
    ylabel("Port Power, [J/s]");
    title("ETank applied");

    figure(5);
    plot(C(:,1), C(:,5));
    xlabel("play time, [s]");
    ylabel("Tank Energy");
    title("ETank applied");