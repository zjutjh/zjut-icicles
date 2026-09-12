package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.entity.AuditLog;
import com.campus.demo.enums.AuditResult;
import com.campus.demo.service.DemoStoreService;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/audit-logs")
public class AuditLogController {

    private final DemoStoreService demoStoreService;

    public AuditLogController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<AuditLog> > listAuditLogs(
            @RequestParam(required = false) Long operatorId,
            @RequestParam(required = false) String actionType,
            @RequestParam(required = false) String targetType,
            @RequestParam(required = false) AuditResult result,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listAuditLogs(operatorId, actionType, targetType, result, pageNo, pageSize));
    }

    @GetMapping("/{logId}")
    public Result<AuditLog> getAuditLog(@PathVariable Long logId) {
        return Result.ok(demoStoreService.getAuditLog(logId));
    }
}
