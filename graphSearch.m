function [Cost, path,init_ID]=graphSearch(initial_states,nodes,segments,obstacle_nodes)

x_pos_init=initial_states(1,:);
y_pos_init=initial_states(2,:);

ID=nodes(:,1);
n_x=nodes(:,2);
n_y=nodes(:,3);

ID_s=segments(:,1);
s_1=segments(:,2);
s_2=segments(:,3);

init_ID=[];
for(q=1:1:6)
    ID_x=find(n_x==x_pos_init(q));
    ID_y=find(n_y==y_pos_init(q));
    for(r=1:1:length(ID_x))
        for(s=1:1:length(ID_y))
        if(ID_x(r)==ID_y(s))
            node_ID=ID_x(r);
            init_ID=[init_ID node_ID];
        end
        end
    end
end

% figure; plot(nodes(:,2), nodes(:,3),'k.');
% hold on;
%     for s = 1:220
%          if (s <= 121) text(nodes(s,2),nodes(s,3),[' ' num2str(s)]); end
%          plot(nodes(segments(s,2:3)',2),nodes(segments(s,2:3)',3),'k');
%     end

Cost=zeros(6);
path(6,6).optimal_path=[];
for(u=1:1:6)
    for(v=1:1:6)
    [d, p] = dijkstra_final(nodes, segments, init_ID(u), init_ID(v),obstacle_nodes);
    Cost(u,v)=d;
    path(u,v).optimal_path=[p];
%         for t = 2:length(p)
%             plot(nodes(p(t-1:t),2),nodes(p(t-1:t),3),'r-.','linewidth',2);
%         end
%     hold on;

    end
end    
end