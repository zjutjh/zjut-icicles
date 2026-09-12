package com.campus.demo.controller;

import com.campus.demo.common.PageResult;
import com.campus.demo.common.Result;
import com.campus.demo.dto.CreateTeamRequest;
import com.campus.demo.dto.UpdateTeamRequest;
import com.campus.demo.entity.Team;
import com.campus.demo.enums.TeamStatus;
import com.campus.demo.service.DemoStoreService;
import jakarta.validation.Valid;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/teams")
public class TeamController {

    private final DemoStoreService demoStoreService;

    public TeamController(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @GetMapping
    public Result<PageResult<Team> > listTeams(
            @RequestParam(required = false) String keyword,
            @RequestParam(required = false) TeamStatus status,
            @RequestParam(required = false) Integer pageNo,
            @RequestParam(required = false) Integer pageSize) {
        return Result.ok(demoStoreService.listTeams(keyword, status, pageNo, pageSize));
    }

    @GetMapping("/{teamId}")
    public Result<Team> getTeam(@PathVariable Long teamId) {
        return Result.ok(demoStoreService.getTeam(teamId));
    }

    @PostMapping
    public Result<Team> createTeam(@Valid @RequestBody CreateTeamRequest request) {
        return Result.ok(demoStoreService.createTeam(request));
    }

    @PutMapping("/{teamId}")
    public Result<Team> updateTeam(@PathVariable Long teamId, @Valid @RequestBody UpdateTeamRequest request) {
        return Result.ok(demoStoreService.updateTeam(teamId, request));
    }
}
