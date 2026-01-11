#include "ldac.h"

DECLFUNC void set_output_pcm_ldac(
    SFINFO* p_sfinfo, void* pp_pcm[], LDAC_SMPL_FMT_T format, int nlnn) {
  SCALAR* p_time;
  int nchs = p_sfinfo->cfg.ch;
  int nsmpl = npow2_ldac(nlnn);
  if (nchs <= 0) {
    return;
  }
  switch (format) {
    case LDAC_SMPL_FMT_S16:
      for (int ich = 0; ich < nchs; ich++) {
        p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        short* p_pcm = pp_pcm[ich];
        int temp;
        for (int isp = 0; isp < nsmpl; isp++) {
          temp = (int)(floor(p_time[isp] + _scalar(0.5)));
          if (temp < -0x8000) temp = -0x8000;
          if (temp >= 0x7FFF) temp = 0x7FFF;
          p_pcm[isp] = *(short*)(&temp);
        }
      }
      break;
    case LDAC_SMPL_FMT_S24:
      for (int ich = 0; ich < nchs; ich++) {
        p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        char* p_pcm = pp_pcm[ich];
        int temp;
        for (int isp = 0; isp < nsmpl; isp++) {
          temp = (int)(floor(p_time[isp] * _scalar(256.0) + _scalar(0.5)));
          if (temp < -0x800000) temp = -0x800000;
          if (temp >= 0x7FFFFF) temp = 0x7FFFFF;
          p_pcm[(isp * 3) + 0] = ((char*)(&temp))[0];
          p_pcm[(isp * 3) + 1] = ((char*)(&temp))[1];
          p_pcm[(isp * 3) + 2] = ((char*)(&temp))[2];
        }
      }
      break;
    case LDAC_SMPL_FMT_S32:
      for (int ich = 0; ich < nchs; ich++) {
        p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        int* p_pcm = pp_pcm[ich];
        long long temp;
        for (int isp = 0; isp < nsmpl; isp++) {
          temp = (long long)(floor(p_time[isp] * _scalar(65536.0) + _scalar(0.5)));
          if (temp < -0x80000000LL) temp = -0x80000000LL;
          if (temp >= 0x7FFFFFFFLL) temp = 0x7FFFFFFFLL;
          p_pcm[isp] = *(int*)(&temp);
        }
      }
      break;
    case LDAC_SMPL_FMT_F32:
      for (int ich = 0; ich < nchs; ich++) {
        p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        float* p_pcm = pp_pcm[ich];
        float temp;
        for (int isp = 0; isp < nsmpl; isp++) {
          temp = p_time[isp] * (_scalar(1.0)/_scalar(32768.0));
          if (temp < -1.0) temp = -1.0;
          if (temp >= 1.0) temp = 1.0;
          p_pcm[isp] = temp;
        }
      }
      break;
    default:
      break;
  }
}
