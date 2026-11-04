// Loads each DynaBARN gazebo9 plugin and runs its Load() with a null model;
// hooks.cc logs the keyframes it builds.
#include <dlfcn.h>
#include <cstdio>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

void read_vec(const void *v, double *xyz)
{
  auto &vec = *static_cast<const ignition::math::Vector3d *>(v);
  xyz[0] = vec.X(); xyz[1] = vec.Y(); xyz[2] = vec.Z();
}

void read_quat(const void *q, double *wxyz)
{
  auto &quat = *static_cast<const ignition::math::Quaterniond *>(q);
  wxyz[0] = quat.W(); wxyz[1] = quat.X(); wxyz[2] = quat.Y(); wxyz[3] = quat.Z();
}

// With SAMPLE_DT set, print Classic's interpolated position every SAMPLE_DT seconds.
void sample_anim(void *shared_ptr_arg)
{
  const char *dt_env = getenv("SAMPLE_DT");
  if (!dt_env)
    return;
  const double dt = atof(dt_env);
  auto &anim = *static_cast<gazebo::common::PoseAnimationPtr *>(shared_ptr_arg);
  gazebo::common::PoseKeyFrame kf(0);
  for (double t = 0; t <= anim->GetLength() + 1e-9; t += dt)
  {
    anim->SetTime(t);
    anim->GetInterpolatedKeyFrame(kf);
    printf("SAMPLE %.4f %.6f %.6f\n", t, kf.Translation().X(), kf.Translation().Y());
  }
}

int main(int argc, char **argv)
{
  setvbuf(stdout, nullptr, _IOLBF, 0);
  for (int i = 1; i < argc; ++i)
  {
    printf("PLUGIN %s\n", argv[i]);
    void *h = dlopen(argv[i], RTLD_NOW | RTLD_LOCAL);
    if (!h) { printf("ERROR %s\n", dlerror()); continue; }
    auto reg = (gazebo::ModelPlugin *(*)())dlsym(h, "RegisterPlugin");
    if (!reg) { printf("ERROR no RegisterPlugin\n"); continue; }
    gazebo::ModelPlugin *p = reg();
    // Plugins assert on a null model; SetAnimation is hooked, so it is never dereferenced.
    static alignas(16) char fake[1 << 16];
    gazebo::physics::ModelPtr model(reinterpret_cast<gazebo::physics::Model *>(fake),
                                    [](gazebo::physics::Model *) {});
    p->Load(model, sdf::ElementPtr());
    printf("END\n");
    fflush(stdout);
  }
  return 0;
}
