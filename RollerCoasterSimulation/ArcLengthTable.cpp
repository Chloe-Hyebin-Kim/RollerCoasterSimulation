#include "ArcLengthTable.h"

void ArcLengthTable::Sampling(const CatmullRomSpline& spline, int i32SampleCount)
{
    m_arrSamples.clear();
    m_f32TotalLength = 0.0f;

    if (i32SampleCount <= 0)
    {
        return;
    }

    int i32SegmentCount = spline.GetSegmentCount();

    if (i32SegmentCount <= 0)
    {
        return;
    }

    /*****프레임 set *****/

    /*  
         위쪽 (N)
           ^
           |
(B) <----- P ----->
           |
           V
       진행방향(T)
    */
    Vec3 prePosition = spline.GetPoint(0.0f);

    Vec3 preTangent;
    Vec3 preNormal;
    Vec3 preBinormal;

    for (int i32Index = 0; i32Index <= i32SampleCount; ++i32Index)
    {
        float ratio = (float)(i32Index) / (float)(i32SampleCount);
        float param = ratio * (float)(i32SegmentCount);// 전체 spline parameter 범위:0 ~ 개수
        
        Vec3 curPosition = spline.GetPoint(param);
		Vec3 curTangent = NormalizeVec3(spline.GetDerivative(param ));
        Vec3 curNormal;
        Vec3 curBinormal;

        if (i32Index > 0)
        {
            m_f32TotalLength += LengthVec3(curPosition - prePosition);//갱신
        }

        if (i32Index == 0)
        {
            Vec3 worldUp(0.0f, 1.0f, 0.0f);

            //뒤집힘 방지
			if (fabs(DotVec3(curTangent, worldUp)) > 0.95f)//worldUp과 평행한 경우
            {
                worldUp = Vec3(1.0f, 0.0f, 0.0f);//기준 벡터를 x축으로 바꿈
            }
          
            curNormal = OrthogonalizeVec3(worldUp, curTangent);//카메라가 기우는 경우에도 뒤집힘 방지 worldUp에서 T의 성분 제거해서 직교화 해야함!
            curBinormal = NormalizeVec3(CrossVec3(curTangent, curNormal));
            curNormal = NormalizeVec3(CrossVec3(curBinormal, curTangent));
        }
        else
        {
            //if (i32Index > 0)
            //   m_f32TotalLength += LengthVec3(curPosition - prePosition);//갱신

            Vec3 rotationAxis = CrossVec3(preTangent, curTangent);
            float axisLength = LengthVec3(rotationAxis);

			if (axisLength < EPS)//방향 안바뀌면
            {
                //이전꺼 그대로 유지
                curNormal = preNormal;
                curBinormal = preBinormal;
            }
            else
            {
                rotationAxis = rotationAxis / axisLength;

                float dotValue = Clampf(DotVec3(preTangent, curTangent), -1.0f, 1.0f);
                float angle = acos(dotValue);

                curNormal = RotateAroundAxis(preNormal, rotationAxis, angle);
                curBinormal = RotateAroundAxis(preBinormal, rotationAxis, angle);
            }

            curNormal = OrthogonalizeVec3(curNormal, curTangent);
            curBinormal = NormalizeVec3(CrossVec3(curTangent, curNormal));
            curNormal = NormalizeVec3(CrossVec3(curBinormal, curTangent));
        }

        ArcSample sample;
        sample.f32Length = m_f32TotalLength;
        sample.f32Param = param;
        sample.vecPosition = curPosition;
        sample.vecTangent = curTangent;
        sample.vecNormal = curNormal;
        sample.vecBinormal = curBinormal;

        m_arrSamples.push_back(sample);

        // 이전 프레임을 다음 curTangent로!
        prePosition = curPosition;
        preTangent = curTangent;
        preNormal = curNormal;
        preBinormal = curBinormal;
        prePosition = curPosition;
    }
}

ArcSample ArcLengthTable::FrameAtArcLength(const CatmullRomSpline& spline, float s) const
{
    //샘플 보간해서 사용!
    if (m_arrSamples.empty() || m_f32TotalLength <= EPS)
        return ArcSample();

	s = fmod(s, m_f32TotalLength);//계속 순환

	if (s < 0.0f)// 음수면 뒤로 가는 거니까 total length 더해서 양수로 만들어줌!!!!!!!!
        s += m_f32TotalLength;

    vector<ArcSample>::const_iterator itr = lower_bound(m_arrSamples.begin(), m_arrSamples.end(), s, 
		[](const ArcSample& sample, float value)//s보다 처음으로 커지는 샘플 찾기
        { 
            return sample.f32Length < value;
        });

    if (itr == m_arrSamples.begin())
        return m_arrSamples.front();

    if (itr == m_arrSamples.end())
        return m_arrSamples.back();

    const ArcSample& sample2 = *itr;
    const ArcSample& sample1 = *(itr - 1);

    const float denom = max(EPS, sample2.f32Length - sample1.f32Length);
    const float alpha = Clampf((s - sample1.f32Length) / denom, 0.0f, 1.0f);
    const float u = sample1.f32Param * (1.0f - alpha) + sample2.f32Param * alpha;

    ArcSample outArcSample;
    outArcSample.f32Length = s;
    outArcSample.f32Param = u;
    outArcSample.vecPosition = spline.GetPoint(u);
    outArcSample.vecTangent = NormalizeVec3(spline.GetDerivative(u));

    // 보간해서 샘플사용!
    outArcSample.vecNormal = NormalizeVec3(LerpVec3(sample1.vecNormal, sample2.vecNormal, alpha));
    outArcSample.vecNormal = OrthogonalizeVec3(outArcSample.vecNormal, outArcSample.vecTangent); //tangent 방향 성분을 버림
       
    outArcSample.vecBinormal = NormalizeVec3(CrossVec3(outArcSample.vecTangent, outArcSample.vecNormal));
    outArcSample.vecNormal = NormalizeVec3(CrossVec3(outArcSample.vecBinormal, outArcSample.vecTangent));

    return outArcSample;
}

/// <summary>
/// spline parameter 반환
///alpha = (D - D0) / (D1 - D0)
///  u = (1 - alpha)u0 + alpha u1
/// </summary>
/// <param name="f32Len"> 실제 곡선을 따라 이동한 거리 </param>
/// <returns> GetPoint 에 넣을 parameter </returns>
//float ArcLengthTable::CalParamFromLength(float f32Len) const
//{
//    if (m_arrSamples.empty())
//    {
//        return 0.0f;
//    }
//
//    if (m_f32TotalLength <= EPS)//length가 0이면 
//    {
//		return m_arrSamples.front().f32Param;//한 점에 모여있으니까!
//    }
//
//    f32Len = fmod(f32Len, m_f32TotalLength);//뺑뻉이
//
//    if (f32Len < 0.0f)//뒤로가는 경우
//    {
//		f32Len += m_f32TotalLength;
//    }
//
//    int i32Left = 0;
//    int i32Right = GetSamplesArrCount() - 1;
//
//    //이분탐색 (length 기준으로 정렬된 테이블 이분탐색)
//    while (i32Left <= i32Right)
//    {
//        int i32Mid = (i32Left + i32Right) / 2;
//
//        //i32Mid가 f32Len보다 작으면 오른쪽으로, 크면 왼쪽으로
//        if (m_arrSamples[i32Mid].f32Length < f32Len)
//        {
//            i32Left = i32Mid + 1;
//        }
//        else
//        {
//            i32Right = i32Mid - 1;
//        }
//    }
//
//	int i32UpperIndex = i32Left;//f32Len보다 처음으로 커지는 index
//
//	if (i32UpperIndex <= 0)//처음보다 작으면 처음에서 계속
//    {
//        return m_arrSamples.front().f32Param;
//    }
//
//	if (i32UpperIndex >= GetSamplesArrCount())
//    {
//        return m_arrSamples.back().f32Param;
//    }
//
//    const ArcSample& sample1 = m_arrSamples[i32UpperIndex - 1];
//    const ArcSample& sample2 = m_arrSamples[i32UpperIndex];
//
//    float f32Len0 = sample1.f32Length;
//    float f32Len1 = sample2.f32Length;
//
//    float f32Param0 = sample1.f32Param;
//    float f32Param1 = sample2.f32Param;
//
//
//    //alpha = (D - D0) / (D1 - D0)
//   //  u = (1 - alpha)u0 + alpha u1
//    float f32Alpha = 0.0f;
//
//    //두 sample 사이 비율
//	if ((f32Len1 - f32Len0) > EPS)//D1과 D0가 거의 같으면
//    {
//        f32Alpha = ((f32Len - f32Len0) / (f32Len1 - f32Len0));//D0~ D1사이 그 어디쯤?
//    }
//
//    return (float)(f32Param0 * (1.0f - f32Alpha) + f32Param1 * f32Alpha);//비율만큼 보간
//}

bool ArcLengthTable::IsEmpty() const
{
    return m_arrSamples.empty();
}
