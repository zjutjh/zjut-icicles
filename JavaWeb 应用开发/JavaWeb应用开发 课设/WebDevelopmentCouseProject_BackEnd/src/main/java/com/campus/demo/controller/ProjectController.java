package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateProjectRequest;
import com.campus.demo.dto.ProjectStatusUpdateRequest;
import com.campus.demo.dto.UpdateProjectRequest;
import com.campus.demo.entity.Project;
import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.ProjectStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/projects")
public class ProjectController {

    private final DemoStoreService demoStoreService;

    public ProjectController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<Project> > listProjects(
            @RequestParam(required = false) String keyword,
            @RequestParam(required = false) ProjectStatus status,
            @RequestParam(required = false) PriorityLevel priority,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listProjects(keyword, status, priority, pageNo, pageSize));
    }

    @GetMapping("/{projectId}")
    public Result<Project> getProject(@PathVariable Long projectId) {
        return Result.ok(demoStoreService.getProject(projectId));
    }

    @PostMapping
    public Result<Project> createProject(@Valid @RequestBody CreateProjectRequest request) {
        return Result.ok(demoStoreService.createProject(request));
    }

    @PutMapping("/{projectId}")
    public Result<Project> updateProject(@PathVariable Long projectId, @Valid @RequestBody UpdateProjectRequest request) {
        return Result.ok(demoStoreService.updateProject(projectId, request));
    }

    @PatchMapping("/{projectId}/status")
    public Result<Project> updateProjectStatus(@PathVariable Long projectId, @Valid @RequestBody ProjectStatusUpdateRequest request) {
        return Result.ok(demoStoreService.updateProjectStatus(projectId, request));
    }
}
