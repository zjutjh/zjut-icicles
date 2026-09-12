import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { assessmentService } from '@/services/assessment-service'
import type {
  AssessmentListQuery,
  FinalizeAssessmentRequest,
  GenerateAssessmentRequest,
  ScoreAssessmentRequest,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useAssessmentsQuery(params: MaybeRefOrGetter<AssessmentListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.assessments(resolvedParams.value)),
    queryFn: () => assessmentService.getAssessments(resolvedParams.value as AssessmentListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useAssessmentDetailQuery(assessmentId: MaybeRefOrGetter<number | null>) {
  const resolvedAssessmentId = computed(() => toValue(assessmentId))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.assessmentDetail(resolvedAssessmentId.value ?? undefined)),
    queryFn: () => assessmentService.getAssessment(resolvedAssessmentId.value as number),
    enabled: computed(() => Boolean(resolvedAssessmentId.value)),
  })
}

export function useGenerateAssessmentsMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: GenerateAssessmentRequest) => assessmentService.generateAssessments(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['assessments'] }),
  })
}

export function useScoreAssessmentMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ assessmentId, payload }: { assessmentId: number; payload: ScoreAssessmentRequest }) =>
      assessmentService.scoreAssessment(assessmentId, payload),
    onSuccess: (_, variables) => {
      queryClient.invalidateQueries({ queryKey: ['assessments'] })
      queryClient.invalidateQueries({ queryKey: QUERY_KEY.assessmentDetail(variables.assessmentId) })
    },
  })
}

export function useFinalizeAssessmentMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ assessmentId, payload }: { assessmentId: number; payload: FinalizeAssessmentRequest }) =>
      assessmentService.finalizeAssessment(assessmentId, payload),
    onSuccess: (_, variables) => {
      queryClient.invalidateQueries({ queryKey: ['assessments'] })
      queryClient.invalidateQueries({ queryKey: QUERY_KEY.assessmentDetail(variables.assessmentId) })
    },
  })
}
