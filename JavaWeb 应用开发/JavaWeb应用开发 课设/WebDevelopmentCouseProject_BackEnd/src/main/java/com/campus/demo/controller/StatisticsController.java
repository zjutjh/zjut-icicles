package com.campus.demo.controller;

import com.campus.demo.common.Result;
import com.campus.demo.entity.DashboardData;
import com.campus.demo.entity.MonthlyStatistics;
import com.campus.demo.entity.QuarterlyStatistics;
import com.campus.demo.entity.YearlyStatistics;
import com.campus.demo.service.DemoStoreService;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/statistics")
public class StatisticsController {

    private final DemoStoreService demoStoreService;

    public StatisticsController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping("/dashboard")
    public Result<DashboardData> dashboard() {
        return Result.ok(demoStoreService.getDashboardData());
    }

    @GetMapping("/monthly")
    public Result<MonthlyStatistics> monthly(
            @RequestParam(required = false) String month,
            @RequestParam(required = false) Long teamId) {
        return Result.ok(demoStoreService.getMonthlyStatistics(month, teamId));
    }

    @GetMapping("/quarterly")
    public Result<QuarterlyStatistics> quarterly(
            @RequestParam(required = false) String quarter,
            @RequestParam(required = false) Long teamId) {
        return Result.ok(demoStoreService.getQuarterlyStatistics(quarter, teamId));
    }

    @GetMapping("/yearly")
    public Result<YearlyStatistics> yearly(
            @RequestParam(required = false) String year,
            @RequestParam(required = false) Long teamId) {
        return Result.ok(demoStoreService.getYearlyStatistics(year, teamId));
    }
}
