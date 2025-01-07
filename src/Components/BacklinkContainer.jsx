import React from 'react';
import NodeBacklink from './NodeBacklink';
import styled from 'styled-components';
import {
  Container,
  Rows,
  HistoryCount
} from './ContainerStyles';
const Title = styled.div`
  text-align: right;
  color: grey;
  font-weight: 200;
`

function BacklinkContainer(props) {
  const {
    lastNetworkData, 
    activeExpandedNodeId, 
    backlinksToShow,
    expandNode
  } = props;
  const nodeText = activeExpandedNodeId === null ? null : lastNetworkData.nodes.find(node => node.id === activeExpandedNodeId).text;
  const title = nodeText ?  `Backlinks(${nodeText})` : 'Backlinks';
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
        {backlinksToShow.length} shown
      </HistoryCount>
    </Container>
  )
}

export default React.memo(BacklinkContainer);