import type { AxiosRequestConfig } from 'axios'

import type {
  AssessmentListQuery,
  FinalizeAssessmentRequest,
  GenerateAssessmentRequest,
  GenerateAssessmentResult,
  MonthlyAssessment,
  PageResult,
  ScoreAssessmentRequest,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class AssessmentService extends BaseService<AxiosRequestConfig> {
  getAssessments(params: AssessmentListQuery) {
    return this.request<PageResult<MonthlyAssessment>>({
      url: this.genBaseURL('/assessments'),
      method: 'GET',
      params,
    })
  }

  getAssessment(assessmentId: number) {
    return this.request<MonthlyAssessment>({
      url: this.genBaseURL(`/assessments/${assessmentId}`),
      method: 'GET',
    })
  }

  generateAssessments(data: GenerateAssessmentRequest) {
    return this.request<GenerateAssessmentResult>({
      url: this.genBaseURL('/assessments/generate'),
      method: 'POST',
      data,
    })
  }

  scoreAssessment(assessmentId: number, data: ScoreAssessmentRequest) {
    return this.request<MonthlyAssessment>({
      url: this.genBaseURL(`/assessments/${assessmentId}/score`),
      method: 'POST',
      data,
    })
  }

  finalizeAssessment(assessmentId: number, data: FinalizeAssessmentRequest) {
    return this.request<MonthlyAssessment>({
      url: this.genBaseURL(`/assessments/${assessmentId}/finalize`),
      method: 'POST',
      data,
    })
  }
}

export const assessmentService = new AssessmentService({
  request,
})
