import React from 'react';
import NodeBacklink from './NodeBacklink';
import styled from 'styled-components';
import {
  Container,
  Rows,
  HistoryCount,
  Title
} from './ContainerStyles';


function BacklinkContainer(props) {
  const {
    lastNetworkData, 
    activeExpandedNodeId, 
    backlinksToShow,
    expandNode,
  } = props;
  const title = backlinksToShow.length > 0  ?  `Backlinks(${backlinksToShow.length})` : 'Backlinks';
  return (
    <Container> 
      <Title>{title}</Title>
      <Rows>
        {backlinksToShow.map(node => (
          <NodeBacklink
            key={node.id}
            node={node}
            expandNode={expandNode}
          ></NodeBacklink>
        ))}
      </Rows>
      <HistoryCount>
        {/* {backlinksToShow.length} shown */}
        <span>filter</span>
      </HistoryCount>
    </Container>
  )
}

export default React.memo(BacklinkContainer);