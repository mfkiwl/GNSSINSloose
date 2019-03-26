%% 文件夹里应有save_to_js.m, ditu.html, mat数据
%% save_to_js({"第一个文件","第二个文件"},{"颜色1","颜色2"},[尺寸1,尺寸2],[因子1,因子2]);
%% 第一个尺寸最好比第二个尺寸大，这样第二个不会覆盖第一个
save_to_js({"IGTunnel.mat","PvtTunnel.mat"},{"red","blue"},[3,2],[1,1e-7]);

%% 然后在浏览器打开ditu.html
