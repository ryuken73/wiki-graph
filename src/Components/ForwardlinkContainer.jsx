import React from 'react';
import styled from 'styled-components';
import NodeBacklink from './NodeBacklink';
import {
  Container,
  Rows,
  HistoryCount,
  Title
} from './ContainerStyles';

function ForwardlinkContainer(props) {
  const {
    lastNetworkData, 
    activeExpandedNodeId, 
    forwardlinksToShow,
    expandNode
  } = props;
  // const nodeText = activeExpandedNodeId === null ? null : lastNetworkData.nodes.find(node => node.id === activeExpandedNodeId).text;
  // const title = nodeText ?  `Forwardlinks(${nodeText})` : 'Forwardlinks';
  const title = forwardlinksToShow.length > 0  ?  `Forwardlinks(${forwardlinksToShow.length})` : 'Forwardlinks';
  console.log('xx',forwardlinksToShow)
  return (
    <Container>
      <Title>{title}</Title>
      <Rows>
        {forwardlinksToShow.map(node => (
          <NodeBacklink
            key={node.id}
            node={node}
            expandNode={expandNode}
          ></NodeBacklink>
        ))}
      </Rows>
      <HistoryCount>
        {/* {forwardlinksToShow.length} shown */}
        <span>filter</span>
      </HistoryCount>
    </Container>
  )
}

export default React.memo(ForwardlinkContainer)