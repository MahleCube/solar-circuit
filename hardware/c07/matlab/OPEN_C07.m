function result = OPEN_C07
% このフォルダのc07回路図を開く。シミュレーションは実行しない。
folder=fileparts(mfilename('fullpath'));
manifest=jsondecode(fileread(fullfile(folder,'MANIFEST.json')));
for k=1:numel(manifest.files)
    item=manifest.files(k);
    path=fullfile(folder,strrep(item.path,'/',filesep));
    assert(isfile(path)&&strcmp(filehash(path),item.sha256), ...
        'C07:FileChanged','配布ファイルがmanifestと一致しません: %s',item.path);
end
for k=1:numel(manifest.custom_libraries)
    lib=manifest.custom_libraries(k);
    libFolder=fullfile(folder,strrep(lib.folder,'/',filesep));
    if bdIsLoaded(lib.name)
        assert(strcmp(filehash(get_param(lib.name,'FileName')), ...
            filehash(fullfile(libFolder,[lib.name '.slx']))), ...
            'C07:OtherLibrary','異なる内容の同名ライブラリが開いています: %s',lib.name);
    end
    addpath(libFolder,'-begin');
end
for lib={'fl_lib','ee_lib','batteryecm_lib','ltc4364demo_lib','solarprotect_lib'}
    load_system(lib{1});
end
name='solar_pv_protected_c07_circuit';
path=fullfile(folder,'models',[name '.slx']);
if bdIsLoaded(name)
    assert(strcmpi(get_param(name,'FileName'),path), ...
        'C07:OtherModel','同名の別モデルが開いています。作業を保存してそのモデルを閉じてから実行してください。');
else
    load_system(path);
end
open_system(name); set_param(name,'ZoomFactor','FitSystem');
result=struct('model',name,'file',path,'open',get_param(name,'Open'), ...
    'dirty',get_param(name,'Dirty'),'simulation_executed',false);
disp(result);
end

function value=filehash(path)
digest=System.Security.Cryptography.SHA256.Create();
stream=System.IO.File.OpenRead(path);
cleanup=onCleanup(@()stream.Dispose());
bytes=uint8(digest.ComputeHash(stream));
value=lower(reshape(dec2hex(bytes,2).',1,[]));
digest.Dispose();
end
