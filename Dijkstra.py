def dijsktra(samples, edges, edge_length, start):
    visited = {start: 0}
    path = {}
    nodes = set(samples)

    while nodes:
        cur_node = None
        for node in nodes:
            if node in visited:
                if cur_node is None:
                    cur_node = node
                elif visited[node] < visited[cur_node]:
                    cur_node = node

        if cur_node is None:
            break

        nodes.remove(cur_node)
        current_weight = visited[cur_node]

        for edge_node in edges[cur_node]:
            weight = current_weight + edge_length[(cur_node, edge_node)]
            if edge_node not in visited or weight < visited[edge_node]:
                visited[edge_node] = weight
                path[edge_node] = cur_node

    return visited, path








