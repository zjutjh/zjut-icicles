package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.FinalizeAssessmentRequest;
import com.campus.demo.dto.GenerateAssessmentRequest;
import com.campus.demo.dto.ScoreAssessmentRequest;
import com.campus.demo.entity.GenerateAssessmentResult;
import com.campus.demo.entity.MonthlyAssessment;
import com.campus.demo.enums.AssessmentStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/assessments")
public class AssessmentController {

    private final DemoStoreService demoStoreService;

    public AssessmentController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @PostMapping("/generate")
    public Result<GenerateAssessmentResult> generate(@Valid @RequestBody GenerateAssessmentRequest request) {
        return Result.ok(demoStoreService.generateAssessments(request));
    }

    @GetMapping
    public Result<PageResult<MonthlyAssessment> > listAssessments(
            @RequestParam(required = false) String assessMonth,
            @RequestParam(required = false) Long teamId,
            @RequestParam(required = false) Long memberId,
            @RequestParam(required = false) AssessmentStatus status,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listAssessments(assessMonth, teamId, memberId, status, pageNo, pageSize));
    }

    @GetMapping("/{assessmentId}")
    public Result<MonthlyAssessment> getAssessment(@PathVariable Long assessmentId) {
        return Result.ok(demoStoreService.getAssessment(assessmentId));
    }

    @PostMapping("/{assessmentId}/score")
    public Result<MonthlyAssessment> scoreAssessment(
            @PathVariable Long assessmentId,
            @RequestHeader(value = "Authorization", required = false) String authorization,
            @Valid @RequestBody ScoreAssessmentRequest request) {
        return Result.ok(demoStoreService.scoreAssessment(assessmentId, request, authorization));
    }

    @PostMapping("/{assessmentId}/finalize")
    public Result<MonthlyAssessment> finalizeAssessment(
            @PathVariable Long assessmentId,
            @RequestHeader(value = "Authorization", required = false) String authorization,
            @RequestBody(required = false) FinalizeAssessmentRequest request) {
        FinalizeAssessmentRequest actualRequest = request == null ? new FinalizeAssessmentRequest() : request;
        return Result.ok(demoStoreService.finalizeAssessment(assessmentId, actualRequest, authorization));
    }
}
