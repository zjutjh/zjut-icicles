package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateWeeklyReportRequest;
import com.campus.demo.dto.ReviewWeeklyReportRequest;
import com.campus.demo.dto.UpdateWeeklyReportRequest;
import com.campus.demo.entity.WeeklyReport;
import com.campus.demo.enums.WeeklyReportStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/reports")
public class ReportController {

    private final DemoStoreService demoStoreService;

    public ReportController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<WeeklyReport> > listReports(
            @RequestParam(required = false) String weekNo,
            @RequestParam(required = false) Long memberId,
            @RequestParam(required = false) WeeklyReportStatus status,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listReports(weekNo, memberId, status, pageNo, pageSize));
    }

    @GetMapping("/{reportId}")
    public Result<WeeklyReport> getReport(@PathVariable Long reportId) {
        return Result.ok(demoStoreService.getReport(reportId));
    }

    @PostMapping
    public Result<WeeklyReport> createReport(@Valid @RequestBody CreateWeeklyReportRequest request) {
        return Result.ok(demoStoreService.createReport(request));
    }

    @PutMapping("/{reportId}")
    public Result<WeeklyReport> updateReport(@PathVariable Long reportId, @Valid @RequestBody UpdateWeeklyReportRequest request) {
        return Result.ok(demoStoreService.updateReport(reportId, request));
    }

    @PostMapping("/{reportId}/review")
    public Result<WeeklyReport> reviewReport(@PathVariable Long reportId, @Valid @RequestBody ReviewWeeklyReportRequest request) {
        return Result.ok(demoStoreService.reviewReport(reportId, request));
    }
}
