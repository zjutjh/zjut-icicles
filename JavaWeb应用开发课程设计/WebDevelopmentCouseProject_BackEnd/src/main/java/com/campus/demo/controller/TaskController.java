package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateTaskProgressRequest;
import com.campus.demo.dto.CreateTaskRequest;
import com.campus.demo.dto.TaskStatusUpdateRequest;
import com.campus.demo.dto.UpdateTaskRequest;
import com.campus.demo.entity.Task;
import com.campus.demo.entity.TaskProgress;
import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.TaskStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/api/tasks")
public class TaskController {

    private final DemoStoreService demoStoreService;

    public TaskController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<Task> > listTasks(
            @RequestParam(required = false) String weekNo,
            @RequestParam(required = false) Long assigneeId,
            @RequestParam(required = false) Long projectId,
            @RequestParam(required = false) TaskStatus status,
            @RequestParam(required = false) PriorityLevel priority,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listTasks(weekNo, assigneeId, projectId, status, priority, pageNo, pageSize));
    }

    @GetMapping("/{taskId}")
    public Result<Task> getTask(@PathVariable Long taskId) {
        return Result.ok(demoStoreService.getTask(taskId));
    }

    @PostMapping
    public Result<Task> createTask(@Valid @RequestBody CreateTaskRequest request) {
        return Result.ok(demoStoreService.createTask(request));
    }

    @PutMapping("/{taskId}")
    public Result<Task> updateTask(@PathVariable Long taskId, @Valid @RequestBody UpdateTaskRequest request) {
        return Result.ok(demoStoreService.updateTask(taskId, request));
    }

    @PatchMapping("/{taskId}/status")
    public Result<Task> updateTaskStatus(@PathVariable Long taskId, @Valid @RequestBody TaskStatusUpdateRequest request) {
        return Result.ok(demoStoreService.updateTaskStatus(taskId, request));
    }

    @GetMapping("/{taskId}/progress")
    public Result<List<TaskProgress>> listTaskProgress(@PathVariable Long taskId) {
        return Result.ok(demoStoreService.listTaskProgress(taskId));
    }

    @PostMapping("/{taskId}/progress")
    public Result<TaskProgress> addTaskProgress(@PathVariable Long taskId, @Valid @RequestBody CreateTaskProgressRequest request) {
        return Result.ok(demoStoreService.addTaskProgress(taskId, request));
    }
}
