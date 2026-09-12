package com.campus.demo.service.impl;

import com.campus.demo.dto.CreateTeamRequest;
import com.campus.demo.entity.Team;
import com.campus.demo.service.DemoStoreService;
import com.campus.demo.service.TeamService;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class TeamServiceImpl implements TeamService {

    private final DemoStoreService demoStoreService;

    public TeamServiceImpl(DemoStoreService demoStoreService) {
        this.demoStoreService = demoStoreService;
    }

    @Override
    public Team getById(Long id) {
        return demoStoreService.getTeam(id);
    }

    @Override
    public List<Team> listTeams() {
        return demoStoreService.listTeams(null, null, 1, Integer.MAX_VALUE).getRecords();
    }

    @Override
    public Team createTeam(CreateTeamRequest request) {
        return demoStoreService.createTeam(request);
    }
}
