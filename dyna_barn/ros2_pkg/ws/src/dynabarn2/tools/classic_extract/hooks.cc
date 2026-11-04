// Interposes the gazebo9 animation calls made by the DynaBARN plugins, logs
// their arguments, and forwards to the real implementation where needed.
#include <dlfcn.h>
#include <cstdio>
#include <string>

void read_vec(const void *v, double *xyz);   // main.cc
void read_quat(const void *q, double *wxyz); // main.cc
void sample_anim(void *shared_ptr_arg);       // main.cc

#define ANIM_CTOR _ZN6gazebo6common13PoseAnimationC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEdb
#define CREATE_KF _ZN6gazebo6common13PoseAnimation14CreateKeyFrameEd
#define TRANSLATE _ZN6gazebo6common12PoseKeyFrame11TranslationERKN8ignition4math2v47Vector3IdEE
#define ROTATE    _ZN6gazebo6common12PoseKeyFrame8RotationERKN8ignition4math2v410QuaternionIdEE
#define SET_ANIM  _ZN6gazebo7physics6Entity12SetAnimationEN5boost10shared_ptrINS_6common13PoseAnimationEEE
#define STR(x) #x
#define XSTR(x) STR(x)

extern "C" {
void ANIM_CTOR(void *self, const std::string &name, double len, bool loop)
{
  printf("ANIM %s %.17g %d\n", name.c_str(), len, (int)loop);
  using F = void (*)(void *, const std::string &, double, bool);
  static F real = (F)dlsym(RTLD_NEXT, XSTR(ANIM_CTOR));
  real(self, name, len, loop);
}

void *CREATE_KF(void *self, double t)
{
  printf("KEY %.17g\n", t);
  using F = void *(*)(void *, double);
  static F real = (F)dlsym(RTLD_NEXT, XSTR(CREATE_KF));
  return real(self, t);
}

void TRANSLATE(void *self, const void *v)
{
  double p[3];
  read_vec(v, p);
  printf("POS %.17g %.17g %.17g\n", p[0], p[1], p[2]);
  using F = void (*)(void *, const void *);
  static F real = (F)dlsym(RTLD_NEXT, XSTR(TRANSLATE));
  real(self, v);
}

void ROTATE(void *self, const void *q)
{
  double r[4];
  read_quat(q, r);
  printf("ROT %.17g %.17g %.17g %.17g\n", r[0], r[1], r[2], r[3]);
  using F = void (*)(void *, const void *);
  static F real = (F)dlsym(RTLD_NEXT, XSTR(ROTATE));
  real(self, q);
}

// The model pointer is null here, so just record the call.
void SET_ANIM(void *, void *anim)
{
  printf("SET\n");
  sample_anim(anim);
}
}
