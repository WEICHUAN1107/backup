#ifndef VRPROCESS_H
#define VRPROCESS_H

void* vrp_inithandle();

int vrp_releasehandle(void* handle);

int vrp_init(void* handle, float _Komni[9], float _D[4], float _xi, int _oriw, int _orih, float _KNew[9], int _neww, int _newh,
            int _safeborder[4], int _colgridN, int _rowgridN, bool _useviewer, float height);

int vrp_start(void* handle);

int vrp_stop(void* handle);

int vrp_doprocess(void* handle, uchar* _img, long long _timestamp);

int vrp_bundle(void* handle);

int refreshviewer(void* handle);

void vrp_restart(void* handle);


#endif


